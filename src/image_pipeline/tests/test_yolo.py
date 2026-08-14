#!/usr/bin/env python3
"""
YOLO 추론 모듈 테스트 — **모델도 rclpy도 없이** 돕니다.

가중치가 아직 없으니 "검출이 맞는가"는 못 봅니다. 대신 모델이 와도 **여전히
틀릴 수 있는 것**을 잠급니다. 전부 "돌아가는데 조용히 틀린" 쪽입니다:

  - 레터박스를 안 되돌림        -> 세로로 60px 밀린 박스. 뎁스가 배경을 물음
  - 채널-앞 출력을 전치 안 함    -> 그럴듯한 박스 4개가 나옴 (에러 없음)
  - end2end 에 NMS를 또 검      -> 인접한 두 불씨가 하나로 합쳐짐
  - 좌표가 0~1 정규화           -> 모든 불이 화면 좌상단에 몰림
  - 클래스 순서 불일치           -> 'fire'를 'person'으로 발행
  - cx,cy,w,h 를 x1,y1,x2,y2 로 -> 박스가 왼쪽 위로 절반 밀림

합성 텐서를 직접 먹이므로, 실제 모델이 오면 `describe_outputs()`로 shape만
확인하고 `layout`을 못박으면 됩니다.
"""

from __future__ import annotations

import os
import sys
import types

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.yolo import (  # noqa: E402
    Detection,
    YoloDetector,
    decode,
    iou_matrix,
    letterbox,
    make_blob,
    nms,
    normalize_output,
    undo_letterbox,
)


class TestUltralyticsDetectorClassContract:
    class _FakeYOLO:
        names = {0: "fire", 1: "person"}

        def __init__(self, _weights):
            pass

    def test_rejects_configured_names_that_disagree_with_pt_metadata(
            self, monkeypatch):
        monkeypatch.setitem(
            sys.modules, "ultralytics", types.SimpleNamespace(YOLO=self._FakeYOLO))
        from image_pipeline.yolo import UltralyticsDetector

        with pytest.raises(ValueError, match="metadata"):
            UltralyticsDetector("model.pt", class_names=["fire"])

    def test_accepts_exact_model_class_order(self, monkeypatch):
        monkeypatch.setitem(
            sys.modules, "ultralytics", types.SimpleNamespace(YOLO=self._FakeYOLO))
        from image_pipeline.yolo import UltralyticsDetector

        detector = UltralyticsDetector(
            "model.pt", class_names=["fire", "person"])
        assert detector.class_names == ("fire", "person")


# --------------------------------------------------------------- 합성 출력


def v8_output(rows, num_classes=2, channels_first=True):
    """`(1, 4+nc, N)` v8 원시 출력을 만듭니다.

    rows: [(cx, cy, w, h, score, class_id), ...]
    """
    arr = np.zeros((len(rows), 4 + num_classes), np.float32)
    for i, (cx, cy, w, h, score, cid) in enumerate(rows):
        arr[i, :4] = (cx, cy, w, h)
        arr[i, 4 + int(cid)] = score
    return arr.T[None, ...] if channels_first else arr[None, ...]


def end2end_output(rows):
    """`(1, N, 6)` — x1, y1, x2, y2, score, class."""
    return np.array([list(r) for r in rows], np.float32)[None, ...]


# ------------------------------------------------------------------ letterbox


class TestLetterbox:
    def test_keeps_aspect_ratio(self):
        img = np.zeros((480, 640, 3), np.uint8)
        out, info = letterbox(img, (640, 640))
        assert out.shape == (640, 640, 3)
        assert info.scale == pytest.approx(1.0)
        # 4:3 을 1:1 에 넣으면 위아래로 (640-480)/2 = 80px 씩
        assert info.pad_x == 0
        assert info.pad_y == 80

    def test_padding_is_not_black(self):
        """여백을 0으로 채우면 어두운 장면에서 불꽃 대비가 왜곡됩니다."""
        img = np.full((480, 640, 3), 200, np.uint8)
        out, info = letterbox(img, (640, 640))
        assert out[0, 0, 0] == 114
        assert out[info.pad_y + 10, 10, 0] == 200

    def test_downscale_uses_area(self):
        """축소 시 에일리어싱으로 작은 불씨가 사라지지 않아야 합니다."""
        img = np.zeros((480, 640, 3), np.uint8)
        img[240:244, 320:324] = 255          # 4x4 불씨
        out, _ = letterbox(img, (320, 320))
        assert out.max() > 0                  # INTER_NEAREST 라면 사라질 수 있음

    def test_undo_round_trips(self):
        img = np.zeros((480, 640, 3), np.uint8)
        _, info = letterbox(img, (640, 640))
        # 원본 (100, 50)-(200, 150) -> 레터박스 좌표 -> 다시 원본
        src = (100.0, 50.0, 200.0, 150.0)
        lb = (src[0] * info.scale + info.pad_x, src[1] * info.scale + info.pad_y,
              src[2] * info.scale + info.pad_x, src[3] * info.scale + info.pad_y)
        back = undo_letterbox(lb, info)
        assert back == pytest.approx(src, abs=1e-6)

    def test_undo_round_trips_when_scaled(self):
        img = np.zeros((480, 640, 3), np.uint8)
        _, info = letterbox(img, (320, 320))
        assert info.scale == pytest.approx(0.5)
        src = (100.0, 50.0, 200.0, 150.0)
        lb = tuple(v * info.scale + (info.pad_x if i % 2 == 0 else info.pad_y)
                   for i, v in enumerate(src))
        assert undo_letterbox(lb, info) == pytest.approx(src, abs=1e-6)

    def test_skipping_undo_misses_vertically(self):
        """★ 되돌리기를 빼먹으면 얼마나 틀리는지 — 이 오차가 뎁스를 바꿉니다.

        640x480 을 640x640 에 넣으면 위아래 80px 여백이 생깁니다. 되돌리지
        않으면 화면 중앙의 불이 80px 아래에 있는 것으로 나가고, 그 자리는
        불이 아니라 바닥이라 거리가 통째로 달라집니다.
        """
        img = np.zeros((480, 640, 3), np.uint8)
        _, info = letterbox(img, (640, 640))
        lb_box = (300.0, 300.0, 340.0, 340.0)
        raw_center_y = (lb_box[1] + lb_box[3]) / 2.0
        undone = undo_letterbox(lb_box, info)
        true_center_y = (undone[1] + undone[3]) / 2.0
        assert raw_center_y - true_center_y == pytest.approx(80.0, abs=1e-6)

    def test_undo_clips_to_source(self):
        """화면 끝의 불은 박스가 밖으로 나갑니다. 안 자르면 빈 영역을 샘플링."""
        img = np.zeros((480, 640, 3), np.uint8)
        _, info = letterbox(img, (640, 640))
        out = undo_letterbox((-50.0, 0.0, 700.0, 640.0), info)
        assert out[0] == 0.0 and out[2] == 640.0
        assert 0.0 <= out[1] <= 480.0 and 0.0 <= out[3] <= 480.0

    def test_blob_shape_and_range(self):
        img = np.full((640, 640, 3), 255, np.uint8)
        blob = make_blob(img)
        assert blob.shape == (1, 3, 640, 640)
        assert blob.max() == pytest.approx(1.0)

    def test_blob_swaps_channels(self):
        """bgr8 을 그대로 넣으면 빨간 불꽃이 파랗게 들어갑니다."""
        img = np.zeros((8, 8, 3), np.uint8)
        img[:, :, 2] = 255                      # BGR 의 R
        assert make_blob(img, swap_rb=True)[0, 0].max() == pytest.approx(1.0)
        assert make_blob(img, swap_rb=False)[0, 0].max() == pytest.approx(0.0)


# ------------------------------------------------------------------- 레이아웃


class TestNormalizeOutput:
    def test_v8_channels_first_is_transposed(self):
        """★ 전치를 빠뜨리면 첫 4행을 검출로 읽어 **그럴듯한 박스 4개**가 나옵니다."""
        raw = v8_output([(10, 10, 4, 4, 0.9, 0)] * 100, num_classes=3)
        assert raw.shape == (1, 7, 100)          # 채널이 앞
        arr, kind = normalize_output(raw, 3)
        assert kind == "v8"
        assert arr.shape == (100, 7)             # 검출이 앞으로 섰는지

    def test_v8_already_row_major(self):
        raw = v8_output([(10, 10, 4, 4, 0.9, 0)] * 100, num_classes=3,
                        channels_first=False)
        arr, kind = normalize_output(raw, 3)
        assert kind == "v8" and arr.shape == (100, 7)

    def test_end2end_detected(self):
        raw = end2end_output([(0, 0, 10, 10, 0.9, 1)] * 12)
        arr, kind = normalize_output(raw, 3)
        assert kind == "end2end" and arr.shape == (12, 6)

    def test_two_classes_is_ambiguous_but_resolved(self):
        """★ nc=2 면 4+nc=6 이라 (N,6)이 양쪽 다 됩니다.

        마지막 열이 정수 클래스 번호처럼 보이는지로 가릅니다.
        """
        e2e = end2end_output([(0, 0, 10, 10, 0.9, 1), (5, 5, 20, 20, 0.8, 0)])
        arr, kind = normalize_output(e2e, 2)
        assert kind == "end2end"

        v8 = v8_output([(10, 10, 4, 4, 0.9, 0)] * 6, num_classes=2)
        assert v8.shape == (1, 6, 6)            # 정확히 애매한 모양
        arr, kind = normalize_output(v8, 2)
        assert kind == "v8" and arr.shape == (6, 6)

    def test_forced_layout_overrides_heuristic(self):
        raw = end2end_output([(0, 0, 10, 10, 0.9, 1)] * 4)
        _, kind = normalize_output(raw, 2, layout="end2end")
        assert kind == "end2end"

    def test_batch_larger_than_one_raises(self):
        raw = np.zeros((2, 100, 6), np.float32)
        with pytest.raises(ValueError, match="배치"):
            normalize_output(raw, 2)

    def test_bad_layout_name_raises(self):
        with pytest.raises(ValueError, match="layout"):
            normalize_output(np.zeros((1, 10, 6), np.float32), 2, layout="v9")


# -------------------------------------------------------------------- 디코딩


class TestDecode:
    def test_v8_converts_xywh_to_xyxy(self):
        """★ cx,cy,w,h 를 x1,y1,x2,y2 로 오해하면 박스가 왼쪽 위로 절반 밀립니다."""
        raw = v8_output([(100, 100, 40, 20, 0.9, 0)] + [(0, 0, 0, 0, 0.0, 0)] * 99,
                        num_classes=2)
        dets, kind = decode(raw, conf=0.5, num_classes=2)
        assert kind == "v8" and len(dets) == 1
        assert dets[0].box == pytest.approx((80.0, 90.0, 120.0, 110.0))

    def test_end2end_keeps_xyxy(self):
        raw = end2end_output([(80, 90, 120, 110, 0.9, 1)])
        dets, kind = decode(raw, conf=0.5, num_classes=2)
        assert kind == "end2end"
        assert dets[0].box == pytest.approx((80.0, 90.0, 120.0, 110.0))
        assert dets[0].class_id == 1

    def test_conf_threshold_filters(self):
        raw = v8_output([(100, 100, 10, 10, 0.9, 0),
                         (200, 200, 10, 10, 0.1, 0)] + [(0, 0, 0, 0, 0.0, 0)] * 98,
                        num_classes=2)
        assert len(decode(raw, conf=0.5, num_classes=2)[0]) == 1
        assert len(decode(raw, conf=0.05, num_classes=2)[0]) == 2

    def test_class_is_argmax_of_scores(self):
        arr = np.zeros((100, 6), np.float32)
        arr[0, :4] = (100, 100, 10, 10)
        arr[0, 4] = 0.3            # class 0
        arr[0, 5] = 0.8            # class 1  <- 이겨야 함
        dets, _ = decode(arr.T[None, ...], conf=0.5, num_classes=2)
        assert len(dets) == 1 and dets[0].class_id == 1
        assert dets[0].score == pytest.approx(0.8)

    def test_class_names_are_applied_in_order(self):
        """★ 순서가 틀리면 불을 사람으로 발행합니다 — 메인이 물을 사람에게 쏩니다."""
        raw = end2end_output([(0, 0, 10, 10, 0.9, 0), (0, 0, 10, 10, 0.9, 1)])
        dets, _ = decode(raw, conf=0.5, class_names=["fire", "person"])
        assert [d.class_name for d in dets] == ["fire", "person"]

    def test_unknown_class_id_becomes_number(self):
        raw = end2end_output([(0, 0, 10, 10, 0.9, 7)])
        dets, _ = decode(raw, conf=0.5, class_names=["fire"], layout="end2end")
        assert dets[0].class_name == "7"

    def test_class_count_mismatch_raises_when_forced(self):
        """모델은 3클래스인데 설정이 2개면 조용히 틀리는 대신 터져야 합니다."""
        raw = v8_output([(10, 10, 4, 4, 0.9, 0)] * 100, num_classes=3)
        with pytest.raises(ValueError, match="class_names"):
            decode(raw, conf=0.5, num_classes=2, layout="v8")

    def test_class_count_mismatch_raises_on_auto(self):
        """자동 판별로 축을 세운 뒤에도 클래스 수가 다르면 터집니다."""
        raw = v8_output([(10, 10, 4, 4, 0.9, 0)] * 100, num_classes=3,
                        channels_first=False)
        with pytest.raises(ValueError, match="클래스 수"):
            decode(raw, conf=0.5, num_classes=2)

    def test_normalized_coordinates_are_rescaled(self):
        """★ 0~1 export. 안 되돌리면 모든 불이 화면 좌상단 1px 안에 몰립니다."""
        raw = end2end_output([(0.1, 0.2, 0.3, 0.4, 0.9, 0)])
        dets, _ = decode(raw, conf=0.5, num_classes=2, input_size=(640, 640))
        assert dets[0].box == pytest.approx((64.0, 128.0, 192.0, 256.0))

    def test_pixel_coordinates_are_left_alone(self):
        raw = end2end_output([(64, 128, 192, 256, 0.9, 0)])
        dets, _ = decode(raw, conf=0.5, num_classes=2, input_size=(640, 640))
        assert dets[0].box == pytest.approx((64.0, 128.0, 192.0, 256.0))

    def test_normalized_no_disables_rescale(self):
        raw = end2end_output([(0.1, 0.2, 0.3, 0.4, 0.9, 0)])
        dets, _ = decode(raw, conf=0.5, num_classes=2, input_size=(640, 640),
                         normalized="no")
        assert dets[0].box == pytest.approx((0.1, 0.2, 0.3, 0.4))

    def test_empty_output_is_not_an_error(self):
        raw = np.zeros((1, 0, 6), np.float32)
        dets, _ = decode(raw, conf=0.5, num_classes=2, layout="end2end")
        assert dets == []


# ----------------------------------------------------------------------- NMS


class TestNms:
    def test_overlapping_same_class_collapses(self):
        a = Detection((0, 0, 100, 100), 0.9, 0)
        b = Detection((5, 5, 105, 105), 0.8, 0)
        assert len(nms([a, b], 0.45)) == 1

    def test_distant_boxes_survive(self):
        a = Detection((0, 0, 50, 50), 0.9, 0)
        b = Detection((300, 300, 350, 350), 0.8, 0)
        assert len(nms([a, b], 0.45)) == 2

    def test_different_classes_survive_by_default(self):
        """★ 불 앞에 선 사람. 클래스 무관 NMS면 둘 중 하나가 사라집니다."""
        fire = Detection((0, 0, 100, 100), 0.9, 0)
        person = Detection((5, 5, 105, 105), 0.8, 1)
        assert len(nms([fire, person], 0.45)) == 2
        assert len(nms([fire, person], 0.45, agnostic=True)) == 1

    def test_keeps_highest_score(self):
        low = Detection((0, 0, 100, 100), 0.3, 0)
        high = Detection((5, 5, 105, 105), 0.95, 0)
        kept = nms([low, high], 0.45)
        assert len(kept) == 1 and kept[0].score == pytest.approx(0.95)

    def test_sorted_by_score(self):
        dets = [Detection((0, 0, 10, 10), 0.3, 0),
                Detection((100, 100, 110, 110), 0.9, 0),
                Detection((200, 200, 210, 210), 0.6, 0)]
        assert [d.score for d in nms(dets, 0.45)] == [0.9, 0.6, 0.3]

    def test_max_det_caps(self):
        dets = [Detection((i * 20, 0, i * 20 + 10, 10), 0.9, 0) for i in range(10)]
        assert len(nms(dets, 0.45, max_det=3)) == 3

    def test_iou_matrix_basics(self):
        boxes = np.array([[0, 0, 10, 10], [0, 0, 10, 10], [20, 20, 30, 30]], np.float32)
        ious = iou_matrix(boxes, np.array([0, 0, 10, 10], np.float32))
        assert ious[0] == pytest.approx(1.0)
        assert ious[1] == pytest.approx(1.0)
        assert ious[2] == pytest.approx(0.0)

    def test_empty_input(self):
        assert nms([], 0.45) == []


# ------------------------------------------------------- 검출기 (가짜 백엔드)


class FakeBackend:
    """미리 정해둔 텐서를 그대로 돌려주는 백엔드. 모델 없이 전 구간을 돕니다."""

    kind = "fake"

    def __init__(self, output):
        self.output = output
        self.calls = 0
        self.last_blob = None

    def infer(self, blob):
        self.calls += 1
        self.last_blob = blob
        return [np.asarray(self.output)]


class TestYoloDetector:
    def _detector(self, output, **kwargs):
        kwargs.setdefault("class_names", ["fire", "person"])
        kwargs.setdefault("conf", 0.5)
        kwargs.setdefault("warmup", False)
        return YoloDetector(FakeBackend(output), **kwargs)

    def test_full_path_returns_source_coordinates(self):
        """레터박스 좌표 -> 원본 640x480 좌표까지 전 구간."""
        # 640x480 -> 640x640, 위아래 80px 여백. 원본 중앙 (320, 240) 은
        # 레터박스에서 (320, 320) 입니다.
        raw = end2end_output([(300, 300, 340, 340, 0.9, 0)])
        det = self._detector(raw, imgsz=640)
        out = det.detect(np.zeros((480, 640, 3), np.uint8))
        assert len(out) == 1
        assert out[0].center() == pytest.approx((320.0, 240.0))
        assert out[0].class_name == "fire"

    def test_end2end_is_not_nms_ed_again(self):
        """★ 인접한 두 불씨. end2end 결과에 NMS를 또 걸면 하나로 합쳐집니다."""
        raw = end2end_output([(300, 300, 340, 340, 0.9, 0),
                              (305, 305, 345, 345, 0.8, 0)])
        det = self._detector(raw, imgsz=640)
        out = det.detect(np.zeros((480, 640, 3), np.uint8))
        assert det.detected_layout == "end2end"
        assert len(out) == 2

    def test_v8_is_nms_ed(self):
        rows = [(320, 320, 40, 40, 0.9, 0), (325, 325, 40, 40, 0.8, 0)]
        rows += [(0, 0, 0, 0, 0.0, 0)] * 98
        det = self._detector(v8_output(rows, num_classes=2), imgsz=640)
        out = det.detect(np.zeros((480, 640, 3), np.uint8))
        assert det.detected_layout == "v8"
        assert len(out) == 1

    def test_degenerate_boxes_are_dropped(self):
        """되돌린 뒤 화면 밖으로 완전히 나간 박스는 버립니다."""
        raw = end2end_output([(300, 0, 340, 20, 0.9, 0)])   # 여백(0~80) 안쪽
        det = self._detector(raw, imgsz=640)
        assert det.detect(np.zeros((480, 640, 3), np.uint8)) == []

    def test_timings_are_recorded(self):
        raw = end2end_output([(300, 300, 340, 340, 0.9, 0)])
        det = self._detector(raw, imgsz=640)
        det.detect(np.zeros((480, 640, 3), np.uint8))
        assert det.timings["total"] > 0.0
        assert set(det.timings) == {"pre", "infer", "post", "total"}

    def test_blob_is_square_regardless_of_input(self):
        raw = end2end_output([(300, 300, 340, 340, 0.9, 0)])
        backend = FakeBackend(raw)
        det = YoloDetector(backend, imgsz=320, conf=0.5, class_names=["fire"],
                           warmup=False)
        det.detect(np.zeros((480, 640, 3), np.uint8))
        assert backend.last_blob.shape == (1, 3, 320, 320)

    def test_warmup_calls_backend(self):
        backend = FakeBackend(end2end_output([(0, 0, 1, 1, 0.1, 0)]))
        YoloDetector(backend, imgsz=64, class_names=["fire"], warmup=True)
        assert backend.calls == 1

    def test_conf_is_live_tunable(self):
        rows = [(320, 320, 40, 40, 0.4, 0)] + [(0, 0, 0, 0, 0.0, 0)] * 99
        det = self._detector(v8_output(rows, num_classes=2), imgsz=640, conf=0.5)
        img = np.zeros((480, 640, 3), np.uint8)
        assert det.detect(img) == []
        det.conf = 0.3
        assert len(det.detect(img)) == 1


class TestStubBackend:
    """가중치 없이 배선을 확인하는 경로. 이게 거짓말하면 검증 수단이 사라집니다."""

    def test_lands_on_source_center(self):
        from image_pipeline.yolo import make_detector
        det = make_detector("", backend="stub", imgsz=640, conf=0.5,
                            class_names=["fire"], warmup=False)
        out = det.detect(np.zeros((480, 640, 3), np.uint8))
        assert len(out) == 1
        assert out[0].center() == pytest.approx((320.0, 240.0), abs=0.5)

    def test_lands_on_center_for_odd_sizes(self):
        from image_pipeline.yolo import make_detector
        det = make_detector("", backend="stub", imgsz=320, conf=0.5,
                            class_names=["fire"], warmup=False)
        out = det.detect(np.zeros((315, 420, 3), np.uint8))
        assert out[0].center() == pytest.approx((210.0, 157.5), abs=0.5)

    def test_does_not_touch_the_filesystem(self):
        """모델 경로를 안 읽어야 가중치 없이 돌 수 있습니다."""
        from image_pipeline.yolo import make_detector
        make_detector("/nonexistent/nope.onnx", backend="stub", warmup=False)

    def test_is_not_an_automatic_fallback(self):
        """★ 확장자로는 절대 stub이 선택되면 안 됩니다.

        모델이 없을 때 조용히 stub으로 대체하면 "검출이 이상한 노드"가 되어
        원인을 엉뚱한 데서 찾게 됩니다.
        """
        from image_pipeline.yolo import make_detector
        with pytest.raises(FileNotFoundError):
            make_detector("/nonexistent/model.onnx")


class TestBackendSelection:
    def test_unknown_extension_raises(self):
        from image_pipeline.yolo import make_detector
        with pytest.raises(ValueError, match="백엔드"):
            make_detector("model.bin")

    def test_missing_onnx_says_what_to_do(self):
        from image_pipeline.yolo import make_detector
        with pytest.raises(FileNotFoundError, match="fake_detection_node"):
            make_detector("/nonexistent/model.onnx")

    def test_hailo_is_an_explicit_stub_not_a_silent_pass(self):
        """★ 조용히 통과하면 로봇에서 "검출 0개"로 나타납니다."""
        from image_pipeline.yolo import make_detector
        with pytest.raises(NotImplementedError, match="팀원5"):
            make_detector("model.hef")
