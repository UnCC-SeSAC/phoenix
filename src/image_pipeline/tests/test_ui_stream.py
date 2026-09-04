from __future__ import annotations

import pytest

from image_pipeline.ui_stream import normalize_boxes, stream_size, throttle


class _Position:
    def __init__(self, x, y):
        self.x, self.y = x, y


class _Center:
    def __init__(self, x, y):
        self.position = _Position(x, y)


class _BBox:
    def __init__(self, cx, cy, width, height):
        self.center = _Center(cx, cy)
        self.size_x, self.size_y = width, height


class _Hypothesis:
    def __init__(self, class_id, score):
        self.class_id, self.score = class_id, score


class _Result:
    def __init__(self, class_id, score):
        self.hypothesis = _Hypothesis(class_id, score)


class _Detection:
    def __init__(self, cx, cy, width, height, class_id="fire", score=0.9):
        self.bbox = _BBox(cx, cy, width, height)
        self.results = [_Result(class_id, score)] if class_id is not None else []


def test_stream_size_preserves_aspect_ratio_without_upscaling():
    assert stream_size(1920, 1080, 640) == (640, 360)
    assert stream_size(320, 240, 640) == (320, 240)


def test_normalized_box_matches_source_coordinates():
    box = normalize_boxes([_Detection(320, 180, 64, 36)], 640, 360)[0]
    assert box["cx"] == pytest.approx(0.5)
    assert box["cy"] == pytest.approx(0.5)
    assert box["w"] == pytest.approx(0.1)
    assert box["h"] == pytest.approx(0.1)


def test_empty_detection_frame_clears_overlay():
    assert normalize_boxes([], 640, 360) == []


def test_malformed_dimensions_and_outside_boxes_are_safe():
    assert normalize_boxes([_Detection(10, 10, 4, 4)], 0, 360) == []
    assert normalize_boxes([_Detection(-100, 180, 20, 20)], 640, 360) == []


def test_numeric_class_id_uses_configured_name():
    box = normalize_boxes(
        [_Detection(320, 180, 64, 36, class_id="1")],
        640, 360, ["fire", "person"],
    )[0]
    assert box["class_name"] == "person"


def test_throttle_allows_first_and_elapsed_frames():
    assert throttle(None, 100.0, 8.0)
    assert not throttle(100.0, 100.05, 8.0)
    assert throttle(100.0, 100.2, 8.0)
