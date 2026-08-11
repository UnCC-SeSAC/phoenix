import json
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from uncc_example.vision_detector import VisionDetector


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def warn(self, message):
        pass


def detector(map_point=None):
    node = VisionDetector.__new__(VisionDetector)
    node.latest_camera_info = object()
    node.target_classes = {"person", "fire"}
    node.map_frame = "map"
    node.detection_pub = Publisher()
    node.get_logger = lambda: Logger()
    point = map_point or SimpleNamespace(
        point=SimpleNamespace(x=2.4, y=1.7, z=2.1)
    )
    node._compute_map_position = lambda data: point
    return node


def message(**overrides):
    payload = {
        "class_name": "person",
        "score": 0.93,
        "x": 320,
        "y": 240,
        "depth": 2.1,
        "depth_status": "ok",
        "stamp_sec": 1786340000,
        "stamp_nanosec": 123456789,
    }
    payload.update(overrides)
    return String(data=json.dumps(payload))


@pytest.mark.parametrize("class_name", ["person", "fire"])
def test_target_detection_preserves_confidence_timestamp_and_map_xy(class_name):
    node = detector()

    node.detections_callback(message(class_name=class_name))

    assert len(node.detection_pub.messages) == 1
    payload = json.loads(node.detection_pub.messages[0].data)
    assert payload == {
        "class": class_name,
        "confidence": 0.93,
        "x": 2.4,
        "y": 1.7,
        "frame_id": "map",
        "stamp_sec": 1786340000,
        "stamp_nanosec": 123456789,
    }


def test_smoke_is_dropped():
    node = detector()
    node.detections_callback(message(class_name="smoke"))
    assert node.detection_pub.messages == []


@pytest.mark.parametrize("depth", [0.0, -0.1, float("nan"), float("inf")])
def test_invalid_depth_is_dropped(depth):
    node = detector()
    node.detections_callback(message(depth=depth))
    assert node.detection_pub.messages == []


def test_tf_failure_is_dropped():
    node = detector()
    node._compute_map_position = lambda data: None
    node.detections_callback(message())
    assert node.detection_pub.messages == []


def test_camera_info_absence_is_dropped():
    node = detector()
    node.latest_camera_info = None
    node.detections_callback(message())
    assert node.detection_pub.messages == []


@pytest.mark.parametrize(
    "overrides",
    [
        {"score": float("nan")},
        {"score": 1.1},
        {"x": float("inf")},
        {"stamp_nanosec": 1_000_000_000},
        {"stamp_sec": -1},
    ],
)
def test_invalid_detection_fields_are_dropped(overrides):
    node = detector()
    node.detections_callback(message(**overrides))
    assert node.detection_pub.messages == []


def test_invalid_json_is_dropped():
    node = detector()
    node.detections_callback(String(data="not-json"))
    assert node.detection_pub.messages == []


@pytest.mark.parametrize("depth_status", ["unknown", "bad", None])
def test_unusable_depth_status_is_dropped(depth_status):
    node = detector()
    node.detections_callback(message(depth_status=depth_status))
    assert node.detection_pub.messages == []


def test_invalid_camera_intrinsics_fail_closed():
    node = detector()
    node.camera_info_callback(SimpleNamespace(k=[0.0] * 9))
    assert node.latest_camera_info is None
