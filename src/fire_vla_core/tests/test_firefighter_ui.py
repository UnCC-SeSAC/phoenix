import json
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from fire_vla_core.domain import (
    Action,
    ActionDecision,
    ActionResult,
    ActionResultStatus,
    ActionSubmission,
    ActionSubmissionStatus,
    ActionType,
    ExecutionSource,
    FireEntity,
    FireState,
    PersonEntity,
    Pose2D,
)
from fire_vla_core.orchestrator import DecisionCycle
from fire_vla_core.ros.firefighter_ui_node import (
    FirefighterHTTPServer,
    StatusStore,
    create_mission_payload,
    validate_server_config,
)
from fire_vla_core.status import VLAStatusTracker
from fire_vla_core.validator import ValidationResult
from fire_vla_core.world_model import WorldModel


def make_world():
    world = WorldModel()
    world.set_mission("mission_01", "인명을 우선 확인해")
    world.robot.pose = Pose2D(0.0, 0.0, 0.2)
    world.robot.home_pose = Pose2D(-1.0, -1.0)
    world.robot.navigation_status = "SUCCEEDED"
    world.perception_ready = True
    world.people["person_0001"] = PersonEntity(
        "person_0001", Pose2D(2.0, 1.0), confidence=0.93
    )
    world.fires["fire_0001"] = FireEntity(
        "fire_0001", Pose2D(3.0, -1.0), confidence=0.91
    )
    return world


def test_status_serializes_world_robot_person_fire():
    payload = VLAStatusTracker().create_payload(make_world().create_snapshot())
    world = payload["world_model"]
    assert world["mission"]["id"] == "mission_01"
    assert world["robot"]["pose"] == {"x": 0.0, "y": 0.0, "yaw": 0.2}
    assert world["robot"]["navigation_status"] == "SUCCEEDED"
    assert world["people"][0]["id"] == "person_0001"
    assert world["fires"][0]["id"] == "fire_0001"
    assert world["perception_ready"] is True


def test_status_serializes_decision_reason_validation_and_submission():
    tracker = VLAStatusTracker()
    action = Action("action_0001", ActionType.WAIT, "대기")
    tracker.update(DecisionCycle(
        ActionDecision(ActionType.WAIT, "현 위치에서 대기", None),
        ValidationResult(True, action=action),
        ActionSubmission("action_0001", ActionSubmissionStatus.ACCEPTED, "ok"),
    ))
    payload = tracker.create_payload({})
    assert payload["decision"] == {
        "action": "WAIT", "target": None, "reason": "현 위치에서 대기"
    }
    assert payload["validation"] == {"approved": True, "reason": ""}
    assert payload["submission"] == {"status": "ACCEPTED", "detail": "ok"}


def test_status_keeps_llm_validation_and_blocked_reason_separate():
    tracker = VLAStatusTracker()
    tracker.update(DecisionCycle(
        ActionDecision(ActionType.EXTINGUISH, "LLM 진압 판단", "fire_0001"),
        ValidationResult(False, reason="분사 가능 범위 밖"),
        None,
        "ACTION_VALIDATION_REJECTED: 분사 가능 범위 밖",
    ))
    payload = tracker.create_payload({})
    assert payload["decision"]["reason"] == "LLM 진압 판단"
    assert payload["validation"]["reason"] == "분사 가능 범위 밖"
    assert payload["blocked_reason"].startswith("ACTION_VALIDATION_REJECTED")


def test_empty_deduplicated_cycle_preserves_latest_meaningful_status():
    tracker = VLAStatusTracker()
    tracker.update(DecisionCycle(
        ActionDecision(ActionType.WAIT, "대기", None), None, None, "blocked"
    ))
    tracker.update(DecisionCycle(None, None, None))
    assert tracker.decision["action"] == "WAIT"
    assert tracker.blocked_reason == "blocked"


def test_status_store_returns_copy_not_shared_mutable_state():
    store = StatusStore()
    source = {"world_model": {"people": [{"id": "person_0001"}]}}
    store.update(source)
    source["world_model"]["people"].clear()
    result = store.get()
    result["world_model"]["people"].clear()
    assert store.get()["world_model"]["people"][0]["id"] == "person_0001"


def test_create_mission_payload_trims_text_and_generates_id():
    payload = create_mission_payload("  인명을 우선 확인해.  ")
    assert payload["text"] == "인명을 우선 확인해."
    assert payload["mission_id"].startswith("mission_ui_")


def test_empty_mission_is_rejected():
    with pytest.raises(ValueError, match="비어"):
        create_mission_payload("   ")


@pytest.mark.parametrize("host", ["0.0.0.0", "192.168.0.3", "example.com"])
def test_non_loopback_host_is_rejected(host):
    with pytest.raises(ValueError, match="ui_host"):
        validate_server_config(host, 8080)


@pytest.mark.parametrize("port", [-1, 65536])
def test_invalid_port_is_rejected(port):
    with pytest.raises(ValueError, match="ui_port"):
        validate_server_config("127.0.0.1", port)


@pytest.fixture
def http_server():
    store = StatusStore()
    world = make_world()
    navigation = Action(
        "nav_0001", ActionType.NAVIGATE_TO, "인명을 우선 확인합니다.",
        "person_0001", Pose2D(2.0, 1.0),
    )
    submission = ActionSubmission(
        navigation.action_id, ActionSubmissionStatus.ACCEPTED, "accepted"
    )
    world.apply_submission(navigation, submission)
    world.apply_action_result(ActionResult(
        navigation.action_id, ExecutionSource.NAVIGATION,
        ActionResultStatus.SUCCEEDED, "person_0001",
    ))
    store.update({
        "timestamp": "2026-08-10T00:00:00+00:00",
        "world_model": world.create_snapshot(),
        "decision": {
            "action": "NAVIGATE_TO",
            "target": "person_0001",
            "reason": "인명을 우선 확인합니다.",
        },
        "validation": {"approved": True, "reason": ""},
        "submission": {"status": "ACCEPTED", "detail": "accepted"},
        "blocked_reason": "",
    })
    missions = []

    def submit(text):
        payload = create_mission_payload(text)
        missions.append(payload)
        return payload

    server = FirefighterHTTPServer("127.0.0.1", 0, store, submit)
    server.start()
    yield server, missions
    server.close()


def request(server, path, *, body=None):
    host, port = server.address
    data = None if body is None else body.encode("utf-8")
    req = Request(
        f"http://{host}:{port}{path}", data=data,
        headers={"Content-Type": "application/json"},
    )
    with urlopen(req, timeout=2) as response:
        return response.status, response.headers["Content-Type"], response.read()


def test_http_root_serves_required_v2_panels(http_server):
    server, _ = http_server
    status, content_type, body = request(server, "/")
    html = body.decode("utf-8")
    assert status == 200 and content_type.startswith("text/html")
    for text in (
        "Mission", "Robot Status", "Current Action", "Live Vision",
        "Semantic Map", "Situation Timeline", "Detected Objects",
        "Recent Result", "VLA Decision / Reason", "CONNECTED", "DISCONNECTED",
    ):
        assert text in html
    for excluded in ("cmd_vel", "NavigateToPose", "Pump", "motor control"):
        assert excluded not in html
    for canonical_field in (
        "world.people", "world.fires", "world.current_action",
        "world.last_action", "decision.reason", "current.target_pose",
    ):
        assert canonical_field in html


def test_status_api_returns_canonical_status(http_server):
    server, _ = http_server
    status, content_type, body = request(server, "/api/status")
    payload = json.loads(body)
    assert status == 200 and content_type.startswith("application/json")
    assert payload["world_model"]["people"][0]["id"] == "person_0001"
    assert payload["world_model"]["fires"][0]["id"] == "fire_0001"
    assert payload["world_model"]["last_action"]["action"] == "NAVIGATE_TO"
    assert payload["world_model"]["last_action"]["status"] == "SUCCEEDED"
    assert payload["decision"] == {
        "action": "NAVIGATE_TO",
        "target": "person_0001",
        "reason": "인명을 우선 확인합니다.",
    }


def test_mission_api_publishes_one_canonical_payload(http_server):
    server, missions = http_server
    status, _, body = request(
        server, "/api/mission", body=json.dumps({"text": "인명을 우선 확인해."})
    )
    response = json.loads(body)
    assert status == 202
    assert len(missions) == 1
    assert missions[0] == response
    assert response["text"] == "인명을 우선 확인해."


@pytest.mark.parametrize("body", ["not-json", "[]", '{"text":"  "}', '{"text":null}'])
def test_malformed_mission_request_is_safely_rejected(http_server, body):
    server, missions = http_server
    with pytest.raises(HTTPError) as error:
        request(server, "/api/mission", body=body)
    assert error.value.code == 400
    assert missions == []


def test_navigation_report_and_spray_lifecycle_are_visible_in_snapshot():
    world = make_world()
    report = Action("report_1", ActionType.REPORT_PERSON, "보고", "person_0001")
    spray = Action("spray_1", ActionType.EXTINGUISH, "분사", "fire_0001")
    accepted = lambda action: ActionSubmission(
        action.action_id, ActionSubmissionStatus.ACCEPTED
    )
    world.apply_submission(report, accepted(report))
    world.apply_action_result(ActionResult(
        "report_1", ExecutionSource.REPORT, ActionResultStatus.SUCCEEDED, "person_0001"
    ))
    world.apply_submission(spray, accepted(spray))
    world.apply_action_result(ActionResult(
        "spray_1", ExecutionSource.SPRAY, ActionResultStatus.SUCCEEDED, "fire_0001"
    ))
    tracker = VLAStatusTracker()
    tracker.update(DecisionCycle(
        ActionDecision(ActionType.EXTINGUISH, "화재 진압을 수행합니다.", "fire_0001"),
        ValidationResult(True, action=spray), accepted(spray),
    ))
    payload = tracker.create_payload(world.create_snapshot())
    snapshot = payload["world_model"]
    assert snapshot["robot"]["navigation_status"] == "SUCCEEDED"
    assert snapshot["people"][0]["reported"] is True
    assert snapshot["fires"][0]["state"] == FireState.PENDING_VERIFICATION.value
    assert snapshot["fires"][0]["spray_count"] == 1
    assert snapshot["last_action"]["action"] == "EXTINGUISH"
    assert snapshot["last_action"]["status"] == "SUCCEEDED"
    assert payload["decision"] == {
        "action": "EXTINGUISH",
        "target": "fire_0001",
        "reason": "화재 진압을 수행합니다.",
    }


def test_existing_launches_do_not_force_start_ui():
    from pathlib import Path

    launch_dir = Path(__file__).parents[2] / "fire_vla_bringup" / "launch"
    for name in ("local_mock_vla.launch.py", "topic_bridge_vla.launch.py"):
        assert "firefighter_ui" not in (launch_dir / name).read_text()
