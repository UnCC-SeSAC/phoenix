import json
import time
from datetime import datetime

import pytest
from std_msgs.msg import String

from fire_vla_core.domain import Pose2D
from fire_vla_core.ros.perception_bridge_node import (
    VLAPerceptionBridgeNode,
    to_canonical_observation,
)
from fire_vla_core.ros.perception_normalizer import CanonicalPerceptionNormalizer
from fire_vla_core.world_model import WorldModel


def detection(class_name="person", **overrides):
    now_ns = time.time_ns()
    value = {
        "class": class_name,
        "confidence": 0.93,
        "x": 2.0,
        "y": 1.0,
        "frame_id": "map",
        "stamp_sec": now_ns // 1_000_000_000,
        "stamp_nanosec": now_ns % 1_000_000_000,
    }
    value.update(overrides)
    return value


@pytest.mark.parametrize("class_name", ["person", "fire"])
def test_valid_detection_maps_to_single_canonical_batch(class_name):
    canonical = to_canonical_observation(detection(class_name))

    assert canonical["frame_id"] == "map"
    assert canonical["frame_valid"] is True
    assert canonical["detector_healthy"] is True
    assert len(canonical["detections"]) == 1
    item = canonical["detections"][0]
    assert item == {
        "class_name": class_name,
        "confidence": 0.93,
        "map_position": {"x": 2.0, "y": 1.0},
    }


def test_source_timestamp_is_preserved_as_utc_iso_with_nanoseconds():
    canonical = to_canonical_observation(detection(
        stamp_sec=1786340000,
        stamp_nanosec=123456789,
    ))
    assert canonical["timestamp"].endswith(".123456789+00:00")
    assert datetime.fromisoformat(canonical["timestamp"]).utcoffset() is not None


def test_smoke_is_ignored():
    assert to_canonical_observation(detection("smoke")) is None


@pytest.mark.parametrize(
    "overrides",
    [
        {"confidence": None},
        {"confidence": float("nan")},
        {"confidence": 1.1},
        {"x": float("nan")},
        {"y": float("inf")},
        {"frame_id": "camera_depth_optical_frame"},
        {"stamp_sec": None},
        {"stamp_nanosec": -1},
    ],
)
def test_invalid_or_incomplete_detection_is_rejected(overrides):
    with pytest.raises(ValueError):
        to_canonical_observation(detection(**overrides))


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


def bridge_callback(message_data):
    node = VLAPerceptionBridgeNode.__new__(VLAPerceptionBridgeNode)
    node._publisher = Publisher()
    logger = Logger()
    node.get_logger = lambda: logger
    node._on_detection(String(data=message_data))
    return node._publisher.messages, logger


def test_bridge_callback_publishes_canonical_json():
    messages, _ = bridge_callback(json.dumps(detection()))
    assert len(messages) == 1
    assert json.loads(messages[0].data)["detections"][0]["class_name"] == "person"


@pytest.mark.parametrize("message_data", ["not-json", "[]"])
def test_bridge_callback_safely_drops_malformed_payload(message_data):
    messages, logger = bridge_callback(message_data)
    assert messages == []
    assert logger.warnings


def apply(world, normalizer, raw):
    canonical = to_canonical_observation(raw)
    if canonical is None:
        return
    world.update_observation_batch(normalizer.normalize(canonical))


def test_bridge_to_world_model_preserves_stable_person_id_and_updates_pose():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)
    apply(world, normalizer, detection(x=2.0, y=1.0))
    apply(world, normalizer, detection(confidence=0.92, x=2.05, y=1.03))

    assert list(world.people) == ["person_0001"]
    assert world.people["person_0001"].position == Pose2D(2.05, 1.03)
    assert world.people["person_0001"].confidence == 0.92


def test_bridge_to_world_model_creates_fire_and_ignores_smoke():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)
    apply(world, normalizer, detection("fire", x=3.0, y=-1.0))
    before = world.create_snapshot()
    apply(world, normalizer, detection("smoke", x=4.0, y=2.0))

    assert list(world.fires) == ["fire_0001"]
    assert world.fires["fire_0001"].position == Pose2D(3.0, -1.0)
    assert world.create_snapshot() == before
