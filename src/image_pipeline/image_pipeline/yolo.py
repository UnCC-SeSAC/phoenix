#!/usr/bin/env python3
"""
YOLO26 추론 — **ROS도 torch도 없이** 도는 검출 모듈.

`aodnet.py`와 같은 발상입니다. Raspberry Pi 5 / Hailo 보드에 torch(수백 MB)를
깔지 않기 위해, 기본 백엔드는 **OpenCV DNN + ONNX**입니다. 의존성은
opencv-python, numpy뿐입니다.

    from image_pipeline.yolo import make_detector
    det = make_detector("models/fire_yolo26s.onnx", class_names=["fire", "person"])
    for d in det.detect(bgr_image):
        print(d.class_name, d.score, d.box)

`yolo_node.py`는 이 파일을 **호출만** 합니다 (구조 규칙: 계산은 모듈에,
노드는 배선만). 그래서 모델도 ROS도 없이 pytest로 검증됩니다 —
`tests/test_yolo.py`가 합성 텐서를 직접 먹여서 파싱·좌표 복원·NMS를 잠급니다.

★★ 검증되지 않은 부분 — 실제 모델을 받으면 **여기부터** 확인하세요
----------------------------------------------------------------------
아래 두 가지는 실제 `.onnx`/`.hef`가 없어 **코드로만 대비**해 둔 것입니다.
HANDOVER 7-2가 "출력 텐서 레이아웃은 여전히 팀원5와 합의 대상"이라고 적어둔
바로 그 지점입니다.

  1. **출력 레이아웃.** YOLO26은 NMS-free / DFL 제거로 이전 버전과 출력이
     다르고, **Hailo에서 나오는 출력과 PC에서 `.pt`로 돌린 출력도 다를 수
     있습니다**(지시서 5-3). 여기서는 두 가지를 지원하고 자동 판별합니다:

         "v8"       (1, 4+nc, A)   박스 cx,cy,w,h + 클래스 점수 nc개. NMS 필요
         "end2end"  (1, N, 6)      박스 x1,y1,x2,y2 + score + class. NMS 불필요

     자동 판별은 **휴리스틱**입니다. `layout` 파라미터로 못박을 수 있고,
     실제 모델이 오면 `print_output_report()`로 실제 shape를 먼저 찍어보세요.

  2. **좌표 정규화.** 대부분의 export는 입력 픽셀 단위지만 0~1로 내보내는
     빌드도 있습니다. 그대로 쓰면 **모든 검출이 화면 좌상단 몇 픽셀 안에**
     몰리는데, 노드는 멀쩡히 도니까 "불이 항상 왼쪽 위에 있다"로 보입니다.
     `normalized="auto"`가 이걸 잡아내지만, 실측으로 확인하고 못박으세요.

⚠ 레터박스를 되돌리지 않으면 조용히 틀립니다
--------------------------------------------
모델 입력은 정사각(640x640)이고 우리 영상은 4:3(640x480)입니다. 비율을 맞추려
위아래에 회색 띠를 넣는데(letterbox), **모델이 내는 좌표는 그 띠를 포함한
좌표계**입니다. 되돌리지 않으면 세로로 60px씩 밀립니다 — 화면 중앙 근처에서는
"조금 틀린 것"처럼 보이고, 3.2m에서 뎁스를 샘플링하면 **박스가 불이 아니라
배경을 물어** 거리가 통째로 바뀝니다. `undo_letterbox()`가 그 복원이고
`tests/test_yolo.py::TestLetterbox`가 잠급니다.

박스 표현은 `depth.py`와 같은 `(x1, y1, x2, y2)` 픽셀 튜플입니다.
좌표계는 **입력 이미지 그대로** — `/image_enhanced`를 먹였으면 축소본 좌표로
나갑니다. 원본 `rgb0`로 되돌리는 것은 태스크②(`detection3d.py`)의 몫이니
여기서 미리 되돌리지 마세요. 두 번 되돌아갑니다.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from typing import Sequence

import cv2
import numpy as np

__all__ = [
    "PAD_VALUE", "LAYOUTS", "Detection", "LetterboxInfo",
    "letterbox", "undo_letterbox", "make_blob",
    "normalize_output", "decode", "nms", "iou_matrix",
    "OnnxCvBackend", "OnnxRuntimeBackend", "StubBackend", "HailoBackend",
    "YoloDetector", "UltralyticsDetector",
    "make_detector", "describe_outputs",
]

#: 레터박스 여백 색. ultralytics 기본값과 같은 114 회색.
#: 검정(0)으로 두면 어두운 장면에서 여백이 불꽃 대비를 왜곡합니다.
PAD_VALUE = 114

LAYOUTS = ("auto", "v8", "end2end")

#: end2end 출력의 검출 개수 상한. 이보다 행이 많으면 v8의 앵커 축으로 봅니다.
#: (ultralytics end2end export는 보통 300, v8 앵커는 640 입력에서 8400)
_END2END_MAX_ROWS = 1000


# --------------------------------------------------------------------- 자료형


@dataclass
class Detection:
    """검출 하나. 좌표는 **입력 이미지 픽셀** `(x1, y1, x2, y2)`."""

    box: tuple[float, float, float, float]
    score: float
    class_id: int
    class_name: str = ""

    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.box
        return ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

    def size(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.box
        return (x2 - x1, y2 - y1)


@dataclass(frozen=True)
class LetterboxInfo:
    """레터박스 복원에 필요한 값. `undo_letterbox()`가 이걸로 되돌립니다."""

    scale: float                    # 원본 -> 모델 입력 배율 (비율 유지)
    pad_x: int                      # 왼쪽 여백 (px, 모델 입력 좌표계)
    pad_y: int                      # 위쪽 여백
    src_size: tuple[int, int]       # (w, h) 원본
    dst_size: tuple[int, int]       # (w, h) 모델 입력


# ----------------------------------------------------------------- 전처리


def letterbox(img: np.ndarray, size, pad_value: int = PAD_VALUE):
    """비율을 유지한 채 `size`(w, h)에 맞추고 남는 곳을 채웁니다.

    ★ 그냥 `cv2.resize(img, (640, 640))`로 늘리면 4:3이 1:1로 찌그러집니다.
      모델은 학습 때 레터박스된 영상을 봤으므로 이때 검출률이 떨어지고,
      더 나쁘게는 **박스의 가로세로비가 틀려** 뎁스 샘플 영역이 어긋납니다.

    반환: `(레터박스 이미지, LetterboxInfo)`
    """
    dst_w, dst_h = int(size[0]), int(size[1])
    h, w = img.shape[:2]
    if w <= 0 or h <= 0:
        raise ValueError(f"빈 이미지입니다: {img.shape}")

    scale = min(dst_w / float(w), dst_h / float(h))
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))

    # 축소는 INTER_AREA. INTER_LINEAR로 줄이면 에일리어싱이 생겨 작은 불씨가
    # 사라질 수 있습니다 (preprocess_node.on_image 와 같은 이유).
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(img, (new_w, new_h), interpolation=interp)

    pad_x = (dst_w - new_w) // 2
    pad_y = (dst_h - new_h) // 2

    shape = (dst_h, dst_w, img.shape[2]) if img.ndim == 3 else (dst_h, dst_w)
    out = np.full(shape, pad_value, dtype=img.dtype)
    out[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized

    return out, LetterboxInfo(scale, pad_x, pad_y, (w, h), (dst_w, dst_h))


def undo_letterbox(box, info: LetterboxInfo, clip: bool = True):
    """모델 입력 좌표계의 박스를 **원본 이미지 좌표**로 되돌립니다.

    여백을 빼고 배율로 나눕니다. `clip=True`면 원본 경계로 자릅니다 —
    화면 끝의 불은 박스가 자주 밖으로 나가고, 안 자르면 뎁스 샘플이
    빈 영역을 읽습니다 (`depth.clip_box`와 같은 이유).
    """
    x1, y1, x2, y2 = (float(v) for v in box)
    s = info.scale if info.scale > 0 else 1.0
    x1 = (x1 - info.pad_x) / s
    x2 = (x2 - info.pad_x) / s
    y1 = (y1 - info.pad_y) / s
    y2 = (y2 - info.pad_y) / s
    if clip:
        w, h = info.src_size
        x1 = min(max(x1, 0.0), float(w))
        x2 = min(max(x2, 0.0), float(w))
        y1 = min(max(y1, 0.0), float(h))
        y2 = min(max(y2, 0.0), float(h))
    return (x1, y1, x2, y2)


def make_blob(img: np.ndarray, swap_rb: bool = True,
              scale: float = 1.0 / 255.0) -> np.ndarray:
    """NCHW float32 블롭. **이미 레터박스된 이미지**를 넣으세요.

    `cv2.dnn.blobFromImage`에 size를 주면 비율을 무시하고 늘려버리므로
    여기서는 크기를 건드리지 않습니다.

    `swap_rb`: 우리 파이프라인은 bgr8이고 ultralytics 학습은 RGB입니다.
    이걸 빠뜨리면 **빨간 불꽃이 파랗게** 들어가서 검출률이 조용히 떨어집니다.
    """
    return cv2.dnn.blobFromImage(img, scalefactor=scale, swapRB=bool(swap_rb),
                                 crop=False)


# ----------------------------------------------------------------- 출력 해석


def _as_2d(raw) -> np.ndarray:
    """`(1, A, B)` 또는 `(A, B)` -> `(A, B)` float32."""
    arr = np.asarray(raw)
    if arr.ndim == 3:
        if arr.shape[0] != 1:
            raise ValueError(
                f"배치 크기가 1이 아닙니다: {arr.shape}. 노드는 프레임 1장씩 돌립니다")
        arr = arr[0]
    if arr.ndim != 2:
        raise ValueError(f"2차원 출력을 기대했습니다: shape={arr.shape}")
    return arr.astype(np.float32, copy=False)


def _looks_like_class_column(col: np.ndarray, num_classes: int | None) -> bool:
    """정수에 가깝고 0..nc-1 범위면 클래스 번호 열로 봅니다."""
    if col.size == 0:
        return False
    if not np.allclose(col, np.round(col), atol=1e-3):
        return False
    if col.min() < -0.5:
        return False
    if num_classes is not None and col.max() > num_classes - 0.5:
        return False
    return True


def normalize_output(raw, num_classes: int | None = None,
                     layout: str = "auto") -> tuple[np.ndarray, str]:
    """원시 출력을 `(N, C)` **검출-행** 배열로 세우고 레이아웃을 판정합니다.

    ★ v8 계열 ONNX는 `(1, 4+nc, 앵커)`로 **채널이 앞**에 옵니다. 전치를
      빠뜨리면 첫 4행을 8400개 검출로 읽어서, 말도 안 되는 좌표가 수천 개
      나오는 게 아니라 **그럴듯한 박스 4개**가 나옵니다. 그래서 조용합니다.

    자동 판별 규칙 (실측으로 못박기 전까지의 휴리스틱):
      - 열이 6개이고 행이 {_END2END_MAX_ROWS}개 이하 -> end2end 후보
      - 행 개수가 `4+nc`와 같으면 채널-앞 v8 -> 전치
      - 애매하면(예: nc=2 이면 4+nc=6) 마지막 열이 정수 클래스 번호처럼
        보이는지로 가릅니다
    """
    if layout not in LAYOUTS:
        raise ValueError(f"layout은 {LAYOUTS} 중 하나여야 합니다: {layout!r}")

    arr = _as_2d(raw)
    rows, cols = arr.shape
    expected = None if num_classes is None else 4 + int(num_classes)

    if layout == "v8":
        if expected is not None and rows == expected and cols != expected:
            arr = arr.T
        elif expected is not None and cols != expected and rows != expected:
            raise ValueError(
                f"v8 레이아웃인데 4+nc={expected}인 축이 없습니다: shape={arr.shape}. "
                "class_names 개수가 모델과 다를 수 있습니다")
        elif expected is None and rows < cols:
            arr = arr.T          # 채널이 앵커보다 적다는 상식적 가정
        return arr, "v8"

    if layout == "end2end":
        if cols != 6 and rows == 6:
            arr = arr.T
        if arr.shape[1] != 6:
            raise ValueError(
                f"end2end 레이아웃은 열이 6개여야 합니다: shape={arr.shape}")
        return arr, "end2end"

    # ---- auto ----
    end2end_shape = (cols == 6 and rows <= _END2END_MAX_ROWS)
    v8_channels_first = (expected is not None and rows == expected)

    if end2end_shape and v8_channels_first:
        # nc=2 이면 4+nc=6 이라 (N, 6)이 양쪽 다 됩니다. 마지막 열로 가릅니다.
        if _looks_like_class_column(arr[:, 5], num_classes):
            return arr, "end2end"
        return arr.T, "v8"

    if end2end_shape and not v8_channels_first:
        if expected is not None and cols == expected:
            # (N, 4+nc) 인 전치된 v8 일 수도 있습니다 (nc=2). 클래스 열로 가릅니다.
            if not _looks_like_class_column(arr[:, 5], num_classes):
                return arr, "v8"
        return arr, "end2end"

    if v8_channels_first:
        return arr.T, "v8"

    if expected is not None and cols == expected:
        return arr, "v8"

    if rows == 6 and cols > _END2END_MAX_ROWS:
        # (6, N) end2end 를 전치해 내보낸 빌드
        return arr.T, "end2end"

    # 마지막 수단: 짧은 축을 채널로 봅니다.
    if rows < cols:
        arr = arr.T
    return arr, "v8"


def _rescale_if_normalized(boxes: np.ndarray, input_size,
                           normalized: str = "auto") -> np.ndarray:
    """0~1 좌표로 나오는 export를 입력 픽셀로 되돌립니다.

    ★ 이걸 안 하면 모든 박스가 좌상단 1px 안에 몰립니다. 노드는 정상 동작하고
      검출 개수도 맞아서, 로그만 보면 아무 문제가 없어 보입니다.
    """
    if normalized == "no" or boxes.size == 0:
        return boxes
    if normalized == "auto":
        if float(np.nanmax(np.abs(boxes))) > 1.5:
            return boxes
    w, h = float(input_size[0]), float(input_size[1])
    out = boxes.copy()
    out[:, 0::2] *= w
    out[:, 1::2] *= h
    return out


def decode(raw, *, conf: float = 0.25, num_classes: int | None = None,
           layout: str = "auto", input_size=None,
           normalized: str = "auto", class_names: Sequence[str] = ()
           ) -> tuple[list[Detection], str]:
    """원시 출력 -> `Detection` 목록 (**모델 입력 좌표계**, NMS 전).

    좌표를 원본으로 되돌리는 것은 호출자(`YoloDetector.detect`)가
    `undo_letterbox`로 합니다 — 여기서 하면 레터박스 정보를 모듈이 알아야 하고
    합성 텐서로 테스트하기가 어려워집니다.

    반환: `(검출 목록, 판정된 레이아웃)`
    """
    if num_classes is None and class_names:
        num_classes = len(class_names)

    arr, kind = normalize_output(raw, num_classes, layout)
    if arr.shape[0] == 0:
        return [], kind

    if kind == "end2end":
        boxes = arr[:, :4]
        scores = arr[:, 4]
        classes = arr[:, 5].astype(np.int32)
    else:
        nc = arr.shape[1] - 4
        if nc <= 0:
            raise ValueError(f"클래스 채널이 없습니다: shape={arr.shape}")
        if num_classes is not None and nc != num_classes:
            raise ValueError(
                f"모델의 클래스 수({nc})와 설정({num_classes})이 다릅니다. "
                "class_names를 모델 학습 때 순서 그대로 맞추세요 — 순서가 틀리면 "
                "'fire'를 'person'으로 발행합니다")
        cls_scores = arr[:, 4:]
        classes = np.argmax(cls_scores, axis=1).astype(np.int32)
        scores = cls_scores[np.arange(cls_scores.shape[0]), classes]
        # cx, cy, w, h -> x1, y1, x2, y2
        cx, cy, bw, bh = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
        boxes = np.stack([cx - bw / 2.0, cy - bh / 2.0,
                          cx + bw / 2.0, cy + bh / 2.0], axis=1)

    if input_size is not None:
        boxes = _rescale_if_normalized(boxes, input_size, normalized)

    keep = scores >= float(conf)
    boxes, scores, classes = boxes[keep], scores[keep], classes[keep]

    out: list[Detection] = []
    for box, score, cid in zip(boxes, scores, classes):
        cid = int(cid)
        name = class_names[cid] if 0 <= cid < len(class_names) else str(cid)
        out.append(Detection(tuple(float(v) for v in box), float(score), cid, name))
    return out, kind


# ---------------------------------------------------------------------- NMS


def iou_matrix(boxes: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """`boxes`(N,4) 각각과 `ref`(4)의 IoU."""
    x1 = np.maximum(boxes[:, 0], ref[0])
    y1 = np.maximum(boxes[:, 1], ref[1])
    x2 = np.minimum(boxes[:, 2], ref[2])
    y2 = np.minimum(boxes[:, 3], ref[3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area = np.clip(boxes[:, 2] - boxes[:, 0], 0, None) * \
        np.clip(boxes[:, 3] - boxes[:, 1], 0, None)
    area_ref = max(ref[2] - ref[0], 0) * max(ref[3] - ref[1], 0)
    union = area + area_ref - inter
    return np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0)


def nms(detections: Sequence[Detection], iou_threshold: float = 0.45,
        agnostic: bool = False, max_det: int = 300) -> list[Detection]:
    """겹치는 박스 제거. 점수 내림차순.

    ★ `agnostic=False`(기본)는 **클래스마다 따로** 억제합니다. 불 앞에 선
      사람처럼 서로 겹치는 다른 클래스를 지우지 않기 위해서입니다.
      ultralytics 기본값과 같습니다.

    ★ YOLO26의 end2end 출력은 **이미 NMS가 끝난** 결과입니다. 거기에 또
      걸면 인접한 두 불씨가 하나로 합쳐집니다. `YoloDetector`가 레이아웃을
      보고 자동으로 건너뜁니다.
    """
    if not detections:
        return []
    order = sorted(range(len(detections)), key=lambda i: detections[i].score,
                   reverse=True)
    boxes = np.array([detections[i].box for i in order], dtype=np.float32)
    classes = np.array([detections[i].class_id for i in order], dtype=np.int32)

    keep: list[int] = []
    suppressed = np.zeros(len(order), dtype=bool)
    for i in range(len(order)):
        if suppressed[i]:
            continue
        keep.append(i)
        if len(keep) >= max_det:
            break
        rest = np.arange(i + 1, len(order))
        rest = rest[~suppressed[rest]]
        if rest.size == 0:
            continue
        ious = iou_matrix(boxes[rest], boxes[i])
        hit = ious > float(iou_threshold)
        if not agnostic:
            hit &= (classes[rest] == classes[i])
        suppressed[rest[hit]] = True

    return [detections[order[i]] for i in keep]


# -------------------------------------------------------------------- 백엔드


class OnnxCvBackend:
    """OpenCV DNN + ONNX. **torch·onnxruntime 불필요** — 기본 백엔드입니다.

    onnx_path : ultralytics `model.export(format="onnx")` 산출물
    threads   : OpenCV 스레드 수. Pi 5(4코어)에서 ROS·Hailo 드라이버와 코어를
                나눠 쓰므로 3 정도가 안전합니다. 0이면 건드리지 않음.
    """

    kind = "onnx"

    def __init__(self, onnx_path: str, threads: int = 0):
        if not os.path.exists(onnx_path):
            raise FileNotFoundError(
                f"모델 파일이 없습니다: {onnx_path}\n"
                "  아직 학습 전이면 fake_detection_node 로 태스크②를 돌리세요:\n"
                "    ros2 run image_pipeline fake_detection_node")
        if threads > 0:
            cv2.setNumThreads(int(threads))
        self.net = cv2.dnn.readNetFromONNX(str(onnx_path))
        self.path = str(onnx_path)

    def infer(self, blob: np.ndarray) -> list[np.ndarray]:
        self.net.setInput(blob)
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        if isinstance(outs, np.ndarray):
            return [outs]
        return list(outs)


class OnnxRuntimeBackend:
    """Explicit ONNX Runtime CPU adapter for OpenCV-incompatible models."""

    kind = "onnxruntime"

    def __init__(self, onnx_path: str, threads: int = 0):
        if not os.path.exists(onnx_path):
            raise FileNotFoundError(f"ONNX 모델 파일이 없습니다: {onnx_path}")
        try:
            import onnxruntime as ort
        except ImportError as exc:
            raise ImportError(
                "backend=onnxruntime에는 onnxruntime이 필요합니다: "
                "python3 -m pip install onnxruntime"
            ) from exc
        options = ort.SessionOptions()
        if threads > 0:
            options.intra_op_num_threads = int(threads)
        self.session = ort.InferenceSession(
            str(onnx_path),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        inputs = self.session.get_inputs()
        if len(inputs) != 1 or inputs[0].type != "tensor(float)":
            raise ValueError(
                "단일 float tensor 입력을 기대합니다: "
                f"{[(item.name, item.shape, item.type) for item in inputs]}"
            )
        self.input_name = inputs[0].name
        self.output_names = [item.name for item in self.session.get_outputs()]
        if not self.output_names:
            raise ValueError("ONNX 모델에 output이 없습니다.")
        self.path = str(onnx_path)

    def infer(self, blob: np.ndarray) -> list[np.ndarray]:
        array = np.asarray(blob, dtype=np.float32)
        outputs = self.session.run(
            self.output_names, {self.input_name: array}
        )
        return [np.asarray(output) for output in outputs]


class StubBackend:
    """가중치 없이 **배선만** 확인하기 위한 가짜 백엔드.

    모델 입력의 정중앙에 박스 하나를 놓습니다. 레터박스를 제대로 되돌렸다면
    결과는 **원본 이미지의 정중앙**이 되어야 합니다. 되돌리기를 빠뜨렸다면
    640x480 입력에서 y가 240이 아니라 **320**으로 나옵니다.

        python3 tools/detect_offline.py --stub smoke01.jpg -o out/
        python3 tools/check_yolo_wiring.py

    ★ 이건 검증 도구지 폴백이 아닙니다. `yolo_node`는 모델이 없으면
      `backend:=stub`을 **명시**해야만 뜹니다. 자동으로 대체하면 "검출이
      이상한 노드"가 되어 원인을 엉뚱한 데서 찾게 됩니다.
    """

    kind = "stub"

    def __init__(self, imgsz: int = 640, box: float = 40.0, score: float = 0.87,
                 class_id: int = 0, **_kwargs):
        c = float(imgsz) / 2.0
        h = float(box) / 2.0
        self.output = np.array([[[c - h, c - h, c + h, c + h, score, class_id]]],
                               dtype=np.float32)
        self.calls = 0

    def infer(self, blob: np.ndarray) -> list[np.ndarray]:
        self.calls += 1
        return [self.output]


class HailoBackend:
    """HailoRT adapter for the measured NHWC UINT8 / NMS-by-class HEFs."""

    kind = "hailo"

    def __init__(self, hef_path: str, **_kwargs):
        if not os.path.exists(hef_path):
            raise FileNotFoundError(f"HEF 모델 파일이 없습니다: {hef_path}")
        try:
            from hailo_platform import (FormatType, HEF, InferVStreams,
                                        InputVStreamParams, OutputVStreamParams,
                                        VDevice)
        except ImportError as exc:
            raise ImportError("HailoRT Python runtime(hailo_platform)이 없습니다") from exc
        self.path = str(hef_path)
        self._hef = HEF(self.path)
        self.input_names = list(self._hef.get_sorted_input_names())
        self.output_names = list(self._hef.get_sorted_output_names())
        if len(self.input_names) != 1 or len(self.output_names) != 1:
            raise ValueError("실측된 단일 입력/단일 NMS 출력 HEF만 지원합니다: "
                             f"inputs={self.input_names}, outputs={self.output_names}")
        self._device = VDevice()
        self._device.__enter__()
        try:
            groups = self._device.configure(self._hef)
            if len(groups) != 1:
                raise ValueError(f"HEF network group이 1개가 아닙니다: {len(groups)}")
            self._network_group = groups[0]
            inputs = InputVStreamParams.make_from_network_group(
                self._network_group, quantized=True, format_type=FormatType.UINT8)
            outputs = OutputVStreamParams.make_from_network_group(
                self._network_group, quantized=False, format_type=FormatType.FLOAT32)
            self._activation = self._network_group.activate(
                self._network_group.create_params())
            self._activation.__enter__()
            self._pipeline = InferVStreams(self._network_group, inputs, outputs,
                                           tf_nms_format=True)
            self._pipeline.__enter__()
        except Exception:
            self.close()
            raise

    @staticmethod
    def _nms_to_end2end(raw: np.ndarray) -> np.ndarray:
        """(1, classes, 5, max_det) yxyx NMS -> (1, N, 6) xyxy."""
        arr = np.asarray(raw, dtype=np.float32)
        if arr.ndim != 4 or arr.shape[0] != 1 or arr.shape[2] != 5:
            raise ValueError("HAILO_NMS_BY_CLASS layout (1,C,5,N)을 기대했습니다: "
                             f"actual={arr.shape}")
        rows = []
        for class_id in range(arr.shape[1]):
            boxes = arr[0, class_id].T
            boxes = boxes[boxes[:, 4] > 0.0]
            if boxes.size == 0:
                continue
            converted = np.empty((boxes.shape[0], 6), dtype=np.float32)
            converted[:, :4] = boxes[:, [1, 0, 3, 2]]
            converted[:, 4] = boxes[:, 4]
            converted[:, 5] = float(class_id)
            rows.append(converted)
        result = np.concatenate(rows, axis=0) if rows else np.empty((0, 6), np.float32)
        return result[None, ...]

    def infer(self, blob: np.ndarray) -> list[np.ndarray]:
        arr = np.asarray(blob)
        if arr.ndim != 4 or arr.shape[0] != 1 or arr.shape[1] != 3:
            raise ValueError(f"NCHW RGB batch=1 blob을 기대했습니다: {arr.shape}")
        nhwc = np.transpose(arr, (0, 2, 3, 1))
        nhwc = np.clip(np.rint(nhwc * 255.0), 0, 255).astype(np.uint8)
        outputs = self._pipeline.infer({self.input_names[0]: nhwc})
        missing = [name for name in self.output_names if name not in outputs]
        if missing:
            raise KeyError(f"Hailo output stream이 없습니다: {missing}")
        return [self._nms_to_end2end(outputs[name]) for name in self.output_names]

    def close(self) -> None:
        for name in ("_pipeline", "_activation", "_device"):
            obj = getattr(self, name, None)
            if obj is not None:
                try:
                    obj.__exit__(None, None, None)
                except Exception:
                    pass
                setattr(self, name, None)

    def __del__(self):
        self.close()


# ------------------------------------------------------------------- 검출기


class YoloDetector:
    """전처리 -> 추론 -> 디코딩 -> NMS -> 좌표 복원.

    backend     : `infer(blob) -> list[np.ndarray]` 를 가진 객체
    imgsz       : 모델 입력 한 변 (정사각). **학습 때 값과 같아야** 합니다
    conf / iou  : 점수 임계값 / NMS IoU
    class_names : 학습 때 순서 그대로. 순서가 틀리면 'fire'를 'person'으로 냅니다
    layout      : "auto" | "v8" | "end2end" — 실측 후 못박으세요
    """

    def __init__(self, backend, *, imgsz: int = 640, conf: float = 0.25,
                 iou: float = 0.45, class_names: Sequence[str] = (),
                 layout: str = "auto", normalized: str = "auto",
                 agnostic: bool = False, max_det: int = 300,
                 swap_rb: bool = True, warmup: bool = True):
        self.backend = backend
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        self.class_names = tuple(str(n) for n in class_names)
        self.layout = layout
        self.normalized = normalized
        self.agnostic = bool(agnostic)
        self.max_det = int(max_det)
        self.swap_rb = bool(swap_rb)

        #: 마지막으로 판정된 레이아웃. 노드가 시작 로그에 찍습니다.
        self.detected_layout = layout
        self.timings: dict[str, float] = {"pre": 0.0, "infer": 0.0,
                                          "post": 0.0, "total": 0.0}

        if warmup:
            # 첫 호출은 내부 버퍼 할당 때문에 느립니다. 미리 태워둡니다 —
            # 노드에서 이걸 빼면 첫 프레임이 통째로 밀립니다 (aodnet 과 같은 이유).
            self._warmup()

    def _warmup(self) -> None:
        dummy = np.zeros((self.imgsz, self.imgsz, 3), np.uint8)
        try:
            self.detect(dummy)
        except Exception:  # noqa: BLE001
            # 워밍업 실패는 치명적이지 않습니다. 진짜 문제면 첫 프레임에서 다시 납니다.
            pass

    @property
    def num_classes(self) -> int | None:
        return len(self.class_names) or None

    def detect(self, img: np.ndarray) -> list[Detection]:
        """BGR 이미지 -> `Detection` 목록 (**입력 이미지 좌표계**)."""
        t0 = time.perf_counter()

        lb_img, info = letterbox(img, (self.imgsz, self.imgsz))
        blob = make_blob(lb_img, swap_rb=self.swap_rb)
        t1 = time.perf_counter()

        outs = self.backend.infer(blob)
        t2 = time.perf_counter()

        # 출력이 여러 개면 **가장 큰 2차원 텐서**를 검출 헤드로 봅니다.
        # (일부 export가 보조 출력을 함께 냅니다)
        raw = max(outs, key=lambda a: np.asarray(a).size)

        dets, kind = decode(raw, conf=self.conf, num_classes=self.num_classes,
                            layout=self.layout, input_size=(self.imgsz, self.imgsz),
                            normalized=self.normalized,
                            class_names=self.class_names)
        self.detected_layout = kind

        # ★ end2end 는 이미 NMS가 끝났습니다. 또 걸면 인접한 불씨가 합쳐집니다.
        if kind != "end2end":
            dets = nms(dets, self.iou, agnostic=self.agnostic, max_det=self.max_det)
        elif len(dets) > self.max_det:
            dets = dets[:self.max_det]

        for d in dets:
            d.box = undo_letterbox(d.box, info)

        # 되돌린 뒤 폭이나 높이가 0인 박스는 화면 밖 검출입니다. 남기면
        # 뎁스 샘플이 빈 영역을 읽고 거리 불명으로 조용히 사라집니다.
        dets = [d for d in dets if d.size()[0] > 1.0 and d.size()[1] > 1.0]

        t3 = time.perf_counter()
        self.timings = {"pre": (t1 - t0) * 1000.0,
                        "infer": (t2 - t1) * 1000.0,
                        "post": (t3 - t2) * 1000.0,
                        "total": (t3 - t0) * 1000.0}
        return dets


class UltralyticsDetector:
    """`.pt` 가중치를 그대로 돌리는 **개발 PC 전용** 검출기.

    ★ 로봇에는 올리지 마세요. torch가 수백 MB이고 Pi 5에서 느립니다.
      로봇용은 ONNX(개발) 또는 Hailo(배포)입니다.

    학습 직후 "가중치가 제대로 나왔는지"를 ONNX export 없이 확인하는 용도로
    둡니다. 같은 `.detect()` API라 노드 코드를 안 고쳐도 됩니다.
    """

    def __init__(self, weights: str, *, imgsz: int = 640, conf: float = 0.25,
                 iou: float = 0.45, class_names: Sequence[str] = (), **_kwargs):
        try:
            from ultralytics import YOLO
        except ImportError as e:      # noqa: F841
            raise ImportError(
                "ultralytics 가 없습니다. 개발 PC에서만: pip install ultralytics\n"
                "  로봇에는 .onnx 를 쓰세요 (torch 불필요)") from None
        self.model = YOLO(str(weights))
        self.imgsz = int(imgsz)
        self.conf = float(conf)
        self.iou = float(iou)
        # 모델이 이름을 알고 있으면 그걸 씁니다 — 손으로 적은 순서보다 안전합니다.
        names = getattr(self.model, "names", None)
        if isinstance(names, dict):
            model_class_names = tuple(str(names[i]) for i in sorted(names))
        else:
            model_class_names = tuple(str(name) for name in (names or ()))
        if class_names:
            configured_class_names = tuple(str(n) for n in class_names)
            if model_class_names and configured_class_names != model_class_names:
                raise ValueError(
                    "class_names가 .pt 모델 metadata와 다릅니다: "
                    f"configured={configured_class_names}, model={model_class_names}"
                )
            self.class_names = configured_class_names
        else:
            self.class_names = model_class_names
        self.detected_layout = "ultralytics"
        self.timings = {"pre": 0.0, "infer": 0.0, "post": 0.0, "total": 0.0}

    def detect(self, img: np.ndarray) -> list[Detection]:
        t0 = time.perf_counter()
        res = self.model.predict(img, imgsz=self.imgsz, conf=self.conf,
                                 iou=self.iou, verbose=False)[0]
        out: list[Detection] = []
        for b in res.boxes:
            x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
            cid = int(b.cls[0])
            name = self.class_names[cid] if cid < len(self.class_names) else str(cid)
            out.append(Detection((x1, y1, x2, y2), float(b.conf[0]), cid, name))
        total = (time.perf_counter() - t0) * 1000.0
        self.timings = {"pre": 0.0, "infer": total, "post": 0.0, "total": total}
        return out


def make_detector(model_path: str, *, backend: str = "auto", **kwargs):
    """확장자로 백엔드를 고릅니다: `.onnx` / `.pt` / `.hef`."""
    ext = os.path.splitext(str(model_path))[1].lower()
    if backend == "auto":
        backend = {".onnx": "onnx", ".pt": "ultralytics", ".hef": "hailo"}.get(ext, "")
        if not backend:
            raise ValueError(
                f"확장자로 백엔드를 못 정했습니다: {model_path!r}. "
                "backend=onnx|ultralytics|hailo 를 명시하세요")

    if backend == "stub":
        # 모델 파일을 읽지 않습니다. 배선 확인 전용 (StubBackend 문서 참조).
        kwargs.pop("threads", None)
        imgsz = int(kwargs.get("imgsz", 640))
        return YoloDetector(StubBackend(imgsz), **kwargs)
    if backend == "onnx":
        threads = int(kwargs.pop("threads", 0))
        return YoloDetector(OnnxCvBackend(model_path, threads=threads), **kwargs)
    if backend == "onnxruntime":
        threads = int(kwargs.pop("threads", 0))
        return YoloDetector(
            OnnxRuntimeBackend(model_path, threads=threads), **kwargs
        )
    if backend == "ultralytics":
        kwargs.pop("threads", None)
        kwargs.pop("layout", None)
        kwargs.pop("normalized", None)
        kwargs.pop("agnostic", None)
        kwargs.pop("max_det", None)
        kwargs.pop("swap_rb", None)
        kwargs.pop("warmup", None)
        return UltralyticsDetector(model_path, **kwargs)
    if backend == "hailo":
        threads = int(kwargs.pop("threads", 0))
        return YoloDetector(HailoBackend(model_path, threads=threads), **kwargs)
    raise ValueError(f"모르는 backend: {backend!r}")


def describe_outputs(model_path: str, imgsz: int = 640) -> str:
    """모델의 **실제 출력 shape**를 찍어봅니다 — 레이아웃 합의용 진단 도구.

        python3 -c "from image_pipeline.yolo import describe_outputs; \\
                    print(describe_outputs('models/fire_yolo26s.onnx'))"

    실제 모델을 받으면 **가장 먼저** 이걸 돌리세요. 자동 판별을 믿기 전에
    눈으로 확인하는 게 몇 시간을 아낍니다 (지시서 5-3).
    """
    backend = OnnxCvBackend(model_path)
    blob = make_blob(np.zeros((imgsz, imgsz, 3), np.uint8))
    outs = backend.infer(blob)
    lines = [f"{model_path} (입력 {imgsz}x{imgsz})", f"  출력 {len(outs)}개:"]
    for i, o in enumerate(outs):
        arr = np.asarray(o)
        lines.append(f"    [{i}] shape={arr.shape} dtype={arr.dtype} "
                     f"min={float(arr.min()):.3f} max={float(arr.max()):.3f}")
    big = max(outs, key=lambda a: np.asarray(a).size)
    for nc in (1, 2, 3, 80):
        try:
            arr, kind = normalize_output(big, nc)
            lines.append(f"    nc={nc:<3d} -> layout={kind}, 검출-행 배열 {arr.shape}")
        except ValueError as e:
            lines.append(f"    nc={nc:<3d} -> 불가: {e}")
    lines.append("  ★ 값 범위가 0~1이면 좌표가 정규화된 export입니다 "
                 "(normalized 파라미터 확인)")
    return "\n".join(lines)
