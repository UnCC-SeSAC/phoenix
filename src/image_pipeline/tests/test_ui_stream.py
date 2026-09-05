#!/usr/bin/env python3
"""UI 스트림 계산 테스트 — rclpy 없이 돕니다."""

from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.ui_stream import (  # noqa: E402
    normalize_boxes,
    stream_size,
    throttle,
)


class _Pos:
    def __init__(self, x, y): self.x, self.y = x, y


class _Center:
    def __init__(self, x, y): self.position = _Pos(x, y)


class _BBox:
    def __init__(self, cx, cy, w, h):
        self.center, self.size_x, self.size_y = _Center(cx, cy), w, h


class _Hyp:
    def __init__(self, class_id, score): self.class_id, self.score = class_id, score


class _Result:
    def __init__(self, class_id, score): self.hypothesis = _Hyp(class_id, score)


class _Det:
    def __init__(self, cx, cy, w, h, class_id="fire", score=0.9):
        self.bbox = _BBox(cx, cy, w, h)
        self.results = [_Result(class_id, score)] if class_id is not None else []


class TestStreamSize:
    def test_downscale_keeps_aspect_ratio(self):
        assert stream_size(1920, 1080, 640) == (640, 360)

    def test_never_upscales(self):
        """확대는 대역폭만 늘고 화질은 그대로입니다."""
        assert stream_size(320, 240, 640) == (320, 240)

    def test_zero_max_width_keeps_original(self):
        assert stream_size(640, 480, 0) == (640, 480)

    def test_invalid_source_size_is_not_a_crash(self):
        assert stream_size(0, 480, 640) == (0, 0)


class TestNormalizeBoxes:
    def test_centered_box_maps_to_center(self):
        boxes = normalize_boxes([_Det(320, 180, 64, 36)], 640, 360)
        assert boxes[0]["cx"] == pytest.approx(0.5)
        assert boxes[0]["cy"] == pytest.approx(0.5)
        assert boxes[0]["w"] == pytest.approx(0.1)
        assert boxes[0]["h"] == pytest.approx(0.1)

    def test_empty_detections_give_empty_list(self):
        """검출 0개도 정상 상태입니다 — 빈 리스트를 내야 UI의 옛 박스가 지워집니다."""
        assert normalize_boxes([], 640, 360) == []

    def test_zero_source_size_is_not_a_zero_division(self):
        assert normalize_boxes([_Det(10, 10, 4, 4)], 0, 360) == []

    def test_box_outside_frame_is_clamped(self):
        """자르지 않으면 SVG가 패널 밖까지 선을 그립니다."""
        boxes = normalize_boxes([_Det(0.0, 180, 100, 36)], 640, 360)
        assert boxes[0]["cx"] - boxes[0]["w"] / 2 == pytest.approx(0.0)

    def test_fully_outside_box_is_dropped(self):
        assert normalize_boxes([_Det(-100.0, 180, 20, 20)], 640, 360) == []

    def test_numeric_class_id_maps_through_class_names(self):
        """vision_msgs 3.x의 class_id는 int64라 '0'으로 옵니다 — UI엔 이름이 떠야 합니다."""
        boxes = normalize_boxes(
            [_Det(320, 180, 64, 36, class_id="0")], 640, 360,
            class_names=["fire", "person"],
        )
        assert boxes[0]["class_name"] == "fire"

    def test_out_of_range_class_id_keeps_raw_value(self):
        boxes = normalize_boxes(
            [_Det(320, 180, 64, 36, class_id="7")], 640, 360, class_names=["fire"]
        )
        assert boxes[0]["class_name"] == "7"

    def test_detection_without_results_is_kept_with_zero_score(self):
        boxes = normalize_boxes([_Det(320, 180, 64, 36, class_id=None)], 640, 360)
        assert boxes[0]["class_name"] == "" and boxes[0]["confidence"] == 0.0


class TestThrottle:
    def test_first_frame_always_passes(self):
        assert throttle(None, 100.0, 8.0) is True

    def test_too_soon_is_rejected(self):
        assert throttle(100.0, 100.05, 8.0) is False

    def test_interval_elapsed_passes(self):
        assert throttle(100.0, 100.2, 8.0) is True

    def test_non_positive_fps_disables_throttling(self):
        assert throttle(100.0, 100.0, 0.0) is True