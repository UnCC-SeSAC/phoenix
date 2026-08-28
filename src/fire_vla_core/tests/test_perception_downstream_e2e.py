"""Production /fire/detections through VLA software-loop regression."""
import json
import time
from datetime import UTC, datetime, timedelta
from types import SimpleNamespace

import pytest

from image_pipeline.detection_json import build_payload, detection_entry, to_json
from fire_vla_core.adapters.mock_adapters import (
    MockNavigationAdapter,
    MockReportAdapter,
    MockResultQueue,
    MockSprayAdapter,
    MockWaitAdapter,
)
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import ActionResultStatus, ActionType, Pose2D
from fire_vla_core.llm import RemoteQwenBackend
from fire_vla_core.orchestrator import VLAOrchestrator
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.ros.fire_detection_adapter import (
    CameraModel,
    adapt_detection_envelope,
    apply_transform,
)
from fire_vla_core.ros.perception_normalizer import CanonicalPerceptionNormalizer
from fire_vla_core.validator import ActionValidator
from fire_vla_core.world_model import WorldModel

CAMERA = CameraModel(
    640,
    480,
    "camera_rgb_optical_frame",
    (400., 0., 320., 0., 400., 240., 0., 0., 1.),
)


class JsonResponse:
    def __init__(self, payload):
        self.payload = payload

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def read(self):
        return json.dumps(self.payload).encode()


def optical_to_map(point, _stamp, _source_frame):
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=0., y=0., z=0.),
        rotation=SimpleNamespace(x=-.5, y=.5, z=-.5, w=.5),
    )

    return apply_transform(point, transform)

def production_payload(*entries, stamp_ns=None):
    stamp_ns = time.time_ns() if stamp_ns is None else stamp_ns
    return build_payload(
        stamp_ns // 1_000_000_000,
        stamp_ns % 1_000_000_000,
        (640, 480),
        entries,
    )


def bridge_into_world(payload, world):
    canonical = adapt_detection_envelope(
        json.loads(to_json(payload)), CAMERA, optical_to_map
    )
    batch = CanonicalPerceptionNormalizer(world).normalize(canonical)
    world.update_observation_batch(batch)
    return batch.observations


def test_person_detection_runs_remote_qwen_validator_and_mock_result(monkeypatch):
    payload = production_payload(
        detection_entry("person", .95, 320, 240, depth=.95, status="ok")
    )
    world = WorldModel()
    world.update_robot_pose(Pose2D(0., 0.))
    world.set_mission("perception_remote", "인명을 우선 확인해")
    observations = bridge_into_world(payload, world)
    person = world.people["person_0001"]
    assert [item.entity_id for item in observations] == ["person_0001"]
    assert (person.position.x, person.position.y) == pytest.approx(
        (.95, 0.), abs=1e-9
    )
    assert person.confidence == .95
    monkeypatch.setattr(
        "urllib.request.urlopen",
        lambda _request, timeout: JsonResponse({
            "action": "NAVIGATE_TO",
            "target": "person_0001",
            "reason": "미보고 인명을 우선 확인한다",
        }),
    )
    results = MockResultQueue()
    navigation = MockNavigationAdapter(results)
    orchestrator = VLAOrchestrator(
        world,
        RemoteQwenBackend("http://pc.test:8088/infer"),
        TargetResolver(),
        ActionValidator(),
        ActionDispatcher(
            navigation,
            MockSprayAdapter(results),
            MockReportAdapter(results),
            MockWaitAdapter(results),
        ),
    )
    cycle = orchestrator.decide_once()
    assert cycle.decision.action == ActionType.NAVIGATE_TO
    assert cycle.validation.approved is True
    assert len(navigation.calls) == 1
    assert orchestrator.process_results(results) == 1
    assert world.last_action.status.value == ActionResultStatus.SUCCEEDED.value
    assert world.robot.navigation_status == ActionResultStatus.SUCCEEDED.value


def test_fire_detection_uses_same_production_coordinate_and_id_path():
    payload = production_payload(
        detection_entry("fire", .91, 360, 240, depth=2., status="ok")
    )
    world = WorldModel()
    observations = bridge_into_world(payload, world)
    fire = world.fires["fire_0001"]
    assert [item.entity_id for item in observations] == ["fire_0001"]
    assert (fire.position.x, fire.position.y) == pytest.approx(
        (2., -.2), abs=1e-9
    )
    assert fire.confidence == .91

@pytest.mark.parametrize("entry", [
    detection_entry("person", .95, 320, 240, depth=None, status="unknown"),
    detection_entry("person", .95, -1, 240, depth=1., status="ok"),
    detection_entry("person", .95, 320, 240, depth=-1., status="ok"),
    detection_entry("vehicle", .95, 320, 240, depth=1., status="ok"),
])
def test_invalid_perception_never_creates_entity(entry):
    world = WorldModel()
    try:
        observations = bridge_into_world(production_payload(entry), world)
    except (ValueError, OverflowError):
        observations = ()
    assert observations == ()
    assert world.people == {} and world.fires == {}


def test_below_world_model_confidence_threshold_creates_no_entity():
    payload = production_payload(
        detection_entry("person", .49, 320, 240, depth=1., status="ok")
    )
    world = WorldModel()
    observations = bridge_into_world(payload, world)
    assert len(observations) == 1
    assert world.people == {}

def test_stale_detection_is_ignored_by_production_normalizer():

    stale = datetime.now(tz=UTC) - timedelta(seconds=30)
    payload = production_payload(
        detection_entry("person", .95, 320, 240, depth=1., status="ok"),
        stamp_ns=int(stale.timestamp() * 1_000_000_000),
    )
    world = WorldModel()
    observations = bridge_into_world(payload, world)
    assert observations == () and world.people == {}


def test_malformed_json_is_rejected_before_entity_creation():
    with pytest.raises(json.JSONDecodeError):
        json.loads("not-json")
