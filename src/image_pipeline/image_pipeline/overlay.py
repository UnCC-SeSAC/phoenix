#!/usr/bin/env python3
"""검출 결과를 프레임 위에 그리는 순수 함수들 — **ROS를 import하지 않습니다.**

`detection_overlay_node.py`가 배선만 하고 그리기는 전부 여기 있습니다
(`yolo_node.py` ↔ `yolo.py`, `detection_3d_node.py` ↔ `detection3d.py`와 같은 구조).
덕 타이핑이라 rclpy 없이 pytest로 검증됩니다 — `tests/test_overlay.py`.

박스 좌표는 **YOLO가 받은 그 프레임 기준**입니다. `yolo_node`가 "받은 이미지 그대로"의
좌표를 발행하므로(`yolo_node.py` 문서 참조), 오버레이가 **같은 토픽을 구독하는 한**
되돌리기·배율 보정이 필요 없습니다. 다른 토픽을 그리면 박스가 어긋납니다.

★ 축소는 **그린 뒤에** 합니다 (`scale_frame`)
----------------------------------------------
먼저 축소하고 원본 좌표로 그리면 박스가 화면 밖으로 나갑니다. 순서가 곧 버그입니다.
"""

from __future__ import annotations

import cv2

from image_pipeline.detection_msgs import box_from_bbox, hypothesis

# BGR. 불은 눈에 띄는 주황-빨강, 사람은 초록 — 흑백으로 봐도 밝기가 갈립니다.
CLASS_COLORS = {
    "fire": (0, 80, 255),
    "person": (0, 220, 60),
}
DEFAULT_COLOR = (0, 255, 255)

_FONT = cv2.FONT_HERSHEY_SIMPLEX

# `draw_hud`가 쓰는 좌상단 띠의 높이. `draw_detections`가 라벨을 이 아래로
# 밀어내는 데 씁니다 — 화면 위쪽에 걸린 검출의 confidence 가 HUD 에 가려지면,
# 하필 **가장 확인하고 싶은 숫자**가 사라집니다.
HUD_BAND_PX = 34


def class_color(class_id: str):
    """클래스 이름 -> BGR. 모르는 클래스는 노란색.

    ★ 대소문자를 접습니다. 학습 라벨이 'Fire'인데 여기서 'fire'만 찾으면 색만
    달라지는 게 아니라 "왜 불인데 사람 색이지" 하고 엉뚱한 데를 뒤지게 됩니다.
    """
    return CLASS_COLORS.get(str(class_id).strip().lower(), DEFAULT_COLOR)


def _clip_box(box, width: int, height: int):
    """(x1,y1,x2,y2)를 프레임 안으로 자릅니다. 완전히 밖이면 None.

    화면 밖 좌표를 그대로 `cv2.rectangle`에 넘겨도 예외는 안 나지만, 라벨을
    붙일 자리를 계산할 때 음수가 섞여 글자가 사라집니다.
    """
    x1, y1, x2, y2 = (float(v) for v in box)
    if x1 > x2:
        x1, x2 = x2, x1
    if y1 > y2:
        y1, y2 = y2, y1
    x1 = max(0.0, min(x1, width - 1.0))
    x2 = max(0.0, min(x2, width - 1.0))
    y1 = max(0.0, min(y1, height - 1.0))
    y2 = max(0.0, min(y2, height - 1.0))
    if x2 - x1 < 1.0 or y2 - y1 < 1.0:
        return None
    return (int(round(x1)), int(round(y1)), int(round(x2)), int(round(y2)))


def _best(detection):
    """검출의 최고 신뢰도 가설 -> (class_id, score). `results`가 비면 None."""
    best = None
    for result in getattr(detection, "results", []) or []:
        class_id, score = hypothesis(result)
        if best is None or score > best[1]:
            best = (class_id, score)
    return best


def _label_baseline(y1: int, text_h: int, frame_h: int, top_margin: int) -> int:
    """라벨 글자의 baseline y. 박스 위 -> 박스 안 -> HUD 아래 순으로 물러납니다."""
    if y1 - text_h - 8 >= top_margin:
        ty = y1 - 6                       # 박스 바로 위 (기본)
    else:
        ty = y1 + text_h + 6              # 자리가 없으면 박스 안쪽 위
    if ty - text_h - 4 < top_margin:      # 그래도 HUD에 물리면 그 아래로
        ty = top_margin + text_h + 4
    return int(min(ty, frame_h - 2))


def draw_detections(frame, detections, *, min_score: float = 0.0,
                    thickness: int = 2, top_margin: int = HUD_BAND_PX) -> int:
    """`frame`(BGR ndarray)에 박스와 `"fire 0.87"` 라벨을 그립니다. 그린 개수 반환.

    `detections`는 `vision_msgs/Detection2D`의 리스트지만 덕 타이핑이라
    테스트의 가짜 객체도 그대로 받습니다.

    ★ `bbox.size_x/size_y`는 **크기**지 우하단 좌표가 아닙니다. 그래서 좌표는
    반드시 `detection_msgs.box_from_bbox`로 얻습니다 — 직접 더하면 박스가 두 배가
    되고, 그 화면을 보고 "모델이 이상하다"는 결론을 내리게 됩니다.
    """
    height, width = frame.shape[:2]
    drawn = 0

    for detection in detections or []:
        best = _best(detection)
        if best is None:
            continue
        class_id, score = best
        if score < min_score:
            continue

        box = _clip_box(box_from_bbox(detection.bbox), width, height)
        if box is None:
            continue
        x1, y1, x2, y2 = box
        color = class_color(class_id)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)

        label = f"{class_id} {score:.2f}"
        (tw, th), _ = cv2.getTextSize(label, _FONT, 0.55, 2)
        ty = _label_baseline(y1, th, height, top_margin)
        tx = min(x1, max(0, width - tw - 2))
        # 라벨 배경 — 불꽃 위 흰 화면에 흰 글자가 얹히면 confidence를 못 읽습니다.
        cv2.rectangle(frame, (tx, ty - th - 4), (tx + tw + 4, ty + 4), color, -1)
        cv2.putText(frame, label, (tx + 2, ty), _FONT, 0.55, (0, 0, 0), 2,
                    cv2.LINE_AA)
        drawn += 1

    return drawn


def hud_text(n_drawn: int, n_total: int, fps: float, note: str = "") -> str:
    """좌상단 한 줄. 문자열만 만들어 두어 테스트가 쉽습니다.

    `fps`는 **YOLO가 검출을 내는 속도**입니다. 오버레이 발행은 `max_fps`로
    스로틀되므로 그쪽 속도를 찍으면 항상 10을 가리켜 아무 정보가 없습니다.
    화면에서 보고 싶은 건 "추론이 실시간인가"이니 yolo 라고 못박아 둡니다.
    """
    text = f"det {n_drawn}/{n_total} | yolo {fps:4.1f}fps"
    if note:
        text += f" | {note}"
    return text


def draw_hud(frame, *, n_drawn: int, n_total: int, fps: float,
             note: str = "") -> None:
    """검출이 0개여도 **반드시** 찍습니다.

    이 한 줄이 있어야 "파이프라인은 도는데 검출이 0"과 "영상 자체가 안 옴"이
    화면만 보고 갈립니다. 빈 화면은 두 경우가 똑같이 보입니다.
    """
    text = hud_text(n_drawn, n_total, fps, note)
    (tw, _), _ = cv2.getTextSize(text, _FONT, 0.6, 2)
    # 높이는 글자 폭과 무관하게 HUD_BAND_PX 로 고정합니다 — `draw_detections`가
    # 이 값을 보고 라벨을 비키므로 둘이 어긋나면 안 됩니다.
    cv2.rectangle(frame, (6, 6), (12 + tw, HUD_BAND_PX - 2), (0, 0, 0), -1)
    cv2.putText(frame, text, (9, HUD_BAND_PX - 8), _FONT, 0.6, (0, 220, 255), 2,
                cv2.LINE_AA)


def scale_frame(frame, display_width: int):
    """`display_width`로 가로를 맞춥니다. 0(또는 이미 그 폭)이면 원본 그대로.

    무선으로 원격 PC에 보낼 때 대역폭을 가장 크게 줄이는 손잡이입니다.
    확대는 하지 않습니다 — 없는 정보를 만들어 봐야 대역폭만 먹습니다.
    """
    if display_width <= 0:
        return frame
    height, width = frame.shape[:2]
    if width <= display_width:
        return frame
    scale = display_width / float(width)
    return cv2.resize(frame, (int(display_width), max(1, int(round(height * scale)))),
                      interpolation=cv2.INTER_AREA)
