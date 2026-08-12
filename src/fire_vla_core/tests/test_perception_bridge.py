import json
import math
import time
from datetime import UTC, datetime, timedelta
from types import SimpleNamespace

import pytest
from std_msgs.msg import String

from fire_vla_core.domain import Pose2D
from fire_vla_core.ros.fire_detection_adapter import (
    CameraModel,
    adapt_detection_envelope,
    adapt_health_status,
    apply_transform,
)
from fire_vla_core.ros.perception_bridge_node import VLAPerceptionBridgeNode
from fire_vla_core.ros.perception_normalizer import CanonicalPerceptionNormalizer
from fire_vla_core.world_model import WorldModel


CAMERA = CameraModel(
    width=640,
    height=480,
    frame_id="camera_rgb_optical_frame",
    k=(100.0, 0.0, 320.0, 0.0, 100.0, 240.0, 0.0, 0.0, 1.0),
)


def envelope(*detections, stamp_ns=None):
    stamp_ns = time.time_ns() if stamp_ns is None else stamp_ns
    return {
        "stamp_sec": stamp_ns // 1_000_000_000,
        "stamp_nanosec": stamp_ns % 1_000_000_000,
        "frame_size": [640, 480],
        "detections": list(detections),
    }


def detection(
    class_name="person",
    *,
    score=0.93,
    x=320,
    y=240,
    depth=2.0,
    depth_status="ok",
):
    return {
        "class_name": class_name,
        "score": score,
        "x": x,
        "y": y,
        "depth": depth,
        "depth_status": depth_status,
    }


class FakeTransform:
    def __init__(self):
        self.calls = []

    def __call__(self, point, stamp, source_frame):
        self.calls.append((point, stamp, source_frame))
        # A deterministic camera-optical -> map transform for software-only tests.
        return point[2] + 0.5, -point[0] + 1.0, point[1]


def adapt(payload, transform=None):
    transform = transform or FakeTransform()
    return adapt_detection_envelope(payload, CAMERA, transform)


@pytest.mark.parametrize("class_name", ["person", "fire"])
def test_ok_depth_projects_person_and_fire_and_maps_score_exactly(class_name):
    transform = FakeTransform()
    canonical = adapt(envelope(detection(class_name)), transform)

    assert canonical["frame_id"] == "map"
    assert canonical["frame_valid"] is True
    assert canonical["detector_healthy"] is True
    item = canonical["detections"][0]
    assert item["class_name"] == class_name
    assert item["confidence"] == 0.93
    assert item["map_position"] == {"x": 2.5, "y": 1.0}
    assert item["depth_status"] == "ok"
    assert transform.calls[0][0] == (0.0, 0.0, 2.0)
    assert transform.calls[0][2] == "camera_rgb_optical_frame"


def test_source_timestamp_is_preserved_with_nanosecond_precision_and_used_for_tf():
    transform = FakeTransform()
    payload = envelope(
        detection(),
        stamp_ns=1_786_340_000_123_456_789,
    )
    canonical = adapt(payload, transform)

    assert canonical["timestamp"].endswith(".123456789+00:00")
    assert transform.calls[0][1] == (1_786_340_000, 123_456_789)
    assert transform.calls[0][2] == "camera_rgb_optical_frame"


def test_person_person_fire_batch_is_one_to_one_and_stable_in_world_model():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)
    canonical = adapt(envelope(
        detection("person", x=320, depth=2.0),
        detection("person", x=340, depth=2.0),
        detection("fire", x=300, depth=3.0),
    ))

    batch = normalizer.normalize(canonical)
    world.update_observation_batch(batch)

    assert [item.entity_id for item in batch.observations] == [
        "person_0001",
        "person_0002",
        "fire_0001",
    ]
    assert list(world.people) == ["person_0001", "person_0002"]
    assert list(world.fires) == ["fire_0001"]


def test_smoke_is_ignored_without_adding_a_semantic_entity():
    canonical = adapt(envelope(detection("smoke")))
    assert canonical["detections"] == []


@pytest.mark.parametrize("depth", [None, 2.0])
def test_unknown_depth_is_fail_closed_even_if_numeric(depth):
    transform = FakeTransform()
    canonical = adapt(envelope(detection(depth=depth, depth_status="unknown")), transform)
    assert canonical["detections"] == []
    assert transform.calls == []


@pytest.mark.parametrize(
    "status",
    ["fallback_bottom", "fallback_below", "fallback_ring"],
)
def test_fallback_depth_is_projected_without_promoting_provenance(status):
    canonical = adapt(envelope(detection(depth=1.5, depth_status=status)))
    assert canonical["detections"][0]["depth_status"] == status
    assert canonical["detections"][0]["depth_m"] == 1.5


def test_stale_source_timestamp_is_preserved_then_rejected_by_normalizer():
    stale_ns = int(
        (datetime.now(tz=UTC) - timedelta(seconds=30)).timestamp()
        * 1_000_000_000
    )
    canonical = adapt(envelope(detection(), stamp_ns=stale_ns))
    world = WorldModel()

    batch = CanonicalPerceptionNormalizer(world).normalize(canonical)
    world.update_observation_batch(batch)

    assert batch.observations == ()
    assert world.people == {}


@pytest.mark.parametrize("score", [-0.1, 1.1, float("nan"), float("inf")])
def test_invalid_score_is_rejected(score):
    with pytest.raises(ValueError, match="score"):
        adapt(envelope(detection(score=score)))


@pytest.mark.parametrize("depth", [float("nan"), float("inf"), -1.0, 0.0])
def test_non_finite_or_non_positive_depth_is_rejected(depth):
    with pytest.raises(ValueError, match="depth"):
        adapt(envelope(detection(depth=depth)))


@pytest.mark.parametrize(
    ("state", "healthy"),
    [
        ("ok", True),
        ("stalled", False),
        ("no_input", False),
        ("waiting_camera_info", False),
    ],
)
def test_heartbeat_is_independent_health_observation(state, healthy):
    status = adapt_health_status({
        "stamp_sec": 1_786_340_000,
        "stamp_nanosec": 123,
        "state": state,
        "last_frame_sec": 1_786_339_999,
        "last_frame_nanosec": 999,
        "age_sec": 0.25,
    })
    assert status["detections"] == []
    assert status["detector_healthy"] is healthy
    assert status["frame_valid"] is healthy
    assert status["perception_health"]["state"] == state


def test_empty_detection_event_can_be_healthy():
    canonical = adapt(envelope())
    assert canonical["detections"] == []
    assert canonical["detector_healthy"] is True


def test_apply_transform_handles_rotation_and_translation():
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        rotation=SimpleNamespace(
            x=0.0,
            y=0.0,
            z=math.sqrt(0.5),
            w=math.sqrt(0.5),
        ),
    )
    result = apply_transform((1.0, 0.0, 0.0), transform)
    assert result == pytest.approx((1.0, 3.0, 3.0))


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


def bridge_callback(message_data, *, status=False):
    node = VLAPerceptionBridgeNode.__new__(VLAPerceptionBridgeNode)
    node._publisher = Publisher()
    node._camera = CAMERA
    node._transform_point = FakeTransform()
    logger = Logger()
    node.get_logger = lambda: logger
    callback = node._on_status if status else node._on_detection
    callback(String(data=message_data))
    return node._publisher.messages, logger


def test_bridge_callback_publishes_latest_canonical_json():
    messages, _ = bridge_callback(json.dumps(envelope(detection())))
    assert len(messages) == 1
    assert json.loads(messages[0].data)["detections"][0]["class_name"] == "person"


def test_bridge_callback_publishes_status_separately():
    payload = {
        "stamp_sec": 1_786_340_000,
        "stamp_nanosec": 0,
        "state": "stalled",
    }
    messages, _ = bridge_callback(json.dumps(payload), status=True)
    assert json.loads(messages[0].data)["detector_healthy"] is False


@pytest.mark.parametrize("message_data", ["not-json", "[]"])
def test_bridge_callback_safely_drops_malformed_payload(message_data):
    messages, logger = bridge_callback(message_data)
    assert messages == []
    assert logger.warnings


def test_latest_contract_to_world_model_preserves_stable_ids_and_updates_pose():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)
    first = adapt(envelope(detection(x=320, depth=2.0)))
    world.update_observation_batch(normalizer.normalize(first))
    second = adapt(envelope(detection(score=0.92, x=321, depth=2.0)))
    world.update_observation_batch(normalizer.normalize(second))

    assert list(world.people) == ["person_0001"]
    assert world.people["person_0001"].position == Pose2D(2.5, 0.98)
    assert world.people["person_0001"].confidence == 0.92
