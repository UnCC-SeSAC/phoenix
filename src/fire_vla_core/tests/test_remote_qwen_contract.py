import json
import threading
import urllib.error
import urllib.request
from datetime import timedelta

import pytest

from fire_vla_core.adapters.mock_adapters import (
    MockNavigationAdapter,
    MockReportAdapter,
    MockResultQueue,
    MockSprayAdapter,
    MockWaitAdapter,
)
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import (
    ActionDecision,
    ActionType,
    ObservationBatch,
    Pose2D,
    SemanticObservation,
    utc_now,
    utc_now_iso,
)
from fire_vla_core.llm import (
    ALLOWED_ACTIONS,
    LLMInferenceError,
    LLMOutputError,
    RemoteQwenBackend,
    build_compact_world_model,
    build_qwen_system_prompt,
    parse_action_decision,
)
from fire_vla_core.orchestrator import VLAOrchestrator
from fire_vla_core.qwen_inference_server import create_server
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.validator import ActionValidator
from fire_vla_core.world_model import WorldModel


def make_world(person_distance=0.95, pose_updated_at=None):
    world = WorldModel()
    world.update_robot_pose(Pose2D(0.0, 0.0), pose_updated_at)
    world.set_mission("software_e2e", "인명을 우선 확인해")
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation(
            "person_0001", "person", 0.95,
            Pose2D(person_distance, 0.0), now,
        ),
    )))
    return world


def make_pipeline(world, backend):
    results = MockResultQueue()
    navigation = MockNavigationAdapter(results)
    dispatcher = ActionDispatcher(
        navigation,
        MockSprayAdapter(results),
        MockReportAdapter(results),
        MockWaitAdapter(results),
    )
    return navigation, results, VLAOrchestrator(
        world, backend, TargetResolver(), ActionValidator(), dispatcher
    )


class DecisionBackend:
    def __init__(self, decision):
        self.decision = decision

    def decide(self, mission, world_model):
        return self.decision


def navigation_backend():
    return DecisionBackend(ActionDecision(
        ActionType.NAVIGATE_TO,
        "미보고 인명을 우선 확인한다",
        "person_0001",
    ))


def test_qwen_prompt_states_strict_action_target_contract():
    prompt = build_qwen_system_prompt()
    assert "SEARCH targets an existing unexplored_zones id" in prompt
    assert "Every non-null target must exactly match one of valid_targets" in prompt
    assert "People are reported automatically outside Qwen" in prompt
    assert "blocks_person_route=true" in prompt
    assert "unexplored_zones is non-empty" in prompt
    assert "REPORT_PERSON" not in ALLOWED_ACTIONS
    assert "REPORT_PERSON targets" not in prompt
    assert "12 words or fewer" in prompt
    assert "FIRE_ONLY, PERSON_FIRE, FULL_EXPLORATION" in prompt
    assert "mission_scope, action" in prompt


@pytest.mark.parametrize("payload, expected", [
    ({"mission_scope": "FULL_EXPLORATION", "action": "NAVIGATE_TO", "target": "person_0001", "reason": "접근"}, ActionType.NAVIGATE_TO),
    ({"mission_scope": "FULL_EXPLORATION", "action": "REPORT_PERSON", "target": "person_0001", "reason": "보고"}, ActionType.REPORT_PERSON),
    ({"mission_scope": "FULL_EXPLORATION", "action": "EXTINGUISH", "target": "fire_0001", "reason": "진압"}, ActionType.EXTINGUISH),
    ({"mission_scope": "FULL_EXPLORATION", "action": "SEARCH", "target": "zone_0001", "reason": "탐색"}, ActionType.SEARCH),
    ({"mission_scope": "FULL_EXPLORATION", "action": "RETURN_HOME", "target": None, "reason": "복귀"}, ActionType.RETURN_HOME),
])
def test_representative_outputs_pass_strict_parser(payload, expected):
    assert parse_action_decision(json.dumps(payload)).action == expected


def test_fresh_pose_validates_and_mock_navigation_result_updates_world():
    world = make_world()
    navigation, results, orchestrator = make_pipeline(world, navigation_backend())

    cycle = orchestrator.decide_once()
    assert cycle.validation.approved
    assert len(navigation.calls) == 1
    assert orchestrator.process_results(results) == 1
    assert world.last_action.status.value == "SUCCEEDED"
    assert world.robot.navigation_status == "SUCCEEDED"
    assert world.current_action is None


def test_stale_pose_is_rejected_without_navigation():
    stale = (utc_now() - timedelta(seconds=5)).isoformat()
    world = make_world(pose_updated_at=stale)
    navigation, _, orchestrator = make_pipeline(world, navigation_backend())

    cycle = orchestrator.decide_once()
    assert not cycle.validation.approved
    assert "너무 오래" in cycle.validation.reason
    assert navigation.calls == []


class RawResponse:
    def __init__(self, body):
        self.body = body

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self):
        return self.body

    def close(self):
        pass


@pytest.mark.parametrize("response", [
    RawResponse(b"not-json"),
    RawResponse(json.dumps({
        "action": "SEARCH", "target": None, "reason": "invalid"
    }).encode()),
    RawResponse(json.dumps({
        "action": "DRIVE", "target": None, "reason": "invalid"
    }).encode()),
])
def test_remote_invalid_responses_fail_closed(monkeypatch, response):
    monkeypatch.setattr("urllib.request.urlopen", lambda *a, **k: response)
    with pytest.raises(LLMOutputError):
        RemoteQwenBackend("http://pc.test:8088/infer").decide(
            "mission", make_world().create_snapshot()
        )


def test_remote_http_500_fails_as_inference_error(monkeypatch):
    monkeypatch.setattr(
        "urllib.request.urlopen",
        lambda *a, **k: (_ for _ in ()).throw(
            urllib.error.HTTPError(
                "url", 500, "error", {}, RawResponse(b'{"error":"device lost"}')
            )
        ),
    )
    with pytest.raises(LLMInferenceError, match="device lost"):
        RemoteQwenBackend("http://pc.test:8088/infer").decide(
            "mission", make_world().create_snapshot()
        )


def test_real_http_remote_backend_to_server_and_existing_pipeline():
    server = create_server("127.0.0.1", 0, navigation_backend())
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        remote = RemoteQwenBackend(
            f"http://127.0.0.1:{server.server_port}/infer", timeout_sec=2.0
        )
        world = make_world()
        navigation, results, orchestrator = make_pipeline(world, remote)
        cycle = orchestrator.decide_once()
        assert cycle.decision.action == ActionType.NAVIGATE_TO
        assert cycle.validation.approved
        assert len(navigation.calls) == 1
        assert orchestrator.process_results(results) == 1
        assert world.last_action.status.value == "SUCCEEDED"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_compact_world_model_excludes_raw_and_transport_state():
    payload = make_world().create_snapshot()
    payload.update({
        "raw_image": [1],
        "laser_scan": [2],
        "occupancy_grid": [3],
        "tf_tree": {"map": "odom"},
        "ros_dump": {"nodes": []},
    })
    compact = build_compact_world_model(payload)
    assert not {
        "raw_image", "laser_scan", "occupancy_grid", "tf_tree", "ros_dump"
    } & set(compact)
    assert "mission" not in compact
    assert "last_action" not in compact
    assert "first_seen" not in compact["people"][0]
    assert "last_seen" not in compact["people"][0]
    assert compact["people"][0]["within_report_range"] is False


def test_compact_world_model_omits_reported_people_but_keeps_fire_risk_relation():
    payload = {
        "people": [{
            "id": "person_01", "position": {"x": 1.0, "y": 0.0},
            "state": "REPORTED", "reported": True,
        }],
        "fires": [{
            "id": "fire_01", "position": {"x": 0.5, "y": 0.0},
            "state": "ACTIVE", "blocks_route_to": ["person_01"],
            "threatens_person": True,
            "threatened_person_id": "person_01",
        }],
    }
    compact = build_compact_world_model(payload)
    assert compact["people"] == []
    assert compact["fires"][0]["blocks_person_route"] is True
    assert compact["fires"][0]["threatens_person"] is True
    assert compact["fires"][0]["threatened_person_id"] == "person_01"
    assert "blocks_route_to" not in compact["fires"][0]
