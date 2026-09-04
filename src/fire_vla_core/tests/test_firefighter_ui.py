import json
import time
from pathlib import Path
from unittest.mock import MagicMock
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from fire_vla_core.adapters.mock_adapters import (
    MockNavigationAdapter, MockResultQueue, MockSprayAdapter, MockWaitAdapter,
)
from fire_vla_core.dispatcher import ActionDispatcher

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
    MissionScope,
    ObservationBatch,
    PersonEntity,
    Pose2D,
    SemanticObservation,
    utc_now_iso,
)
from fire_vla_core.orchestrator import DecisionCycle, VLAOrchestrator
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.ros.topic_bridge_person_report_adapter import (
    TopicBridgePersonReportAdapter,
)
from fire_vla_core.ros.firefighter_ui_node import (
    ControlModeOwner,
    FirefighterHTTPServer,
    StatusStore,
    create_mission_payload,
    normalize_mode,
    normalize_rule_based_command,
    validate_server_config,
)
from fire_vla_core.status import VLAStatusTracker
from fire_vla_core.validator import ActionValidator, ValidationResult
from fire_vla_core.world_model import WorldModel, WorldModelConfig


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
        "mission_scope": "FULL_EXPLORATION", "action": "WAIT", "target": None, "reason": "현 위치에서 대기"
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


def test_status_store_keeps_vla_and_rule_based_snapshots_isolated():
    store = StatusStore()
    store.update({"timestamp": "vla", "world_model": {}}, "VLA")
    store.update({"timestamp": "rule", "mode": "RULE_BASED"}, "RULE_BASED")
    assert store.get()["timestamp"] == "vla"
    assert store.get("RULE_BASED")["timestamp"] == "rule"


@pytest.mark.parametrize("value", ["bad", 1, {}, []])
def test_invalid_ui_mode_is_rejected(value):
    with pytest.raises(ValueError, match="mode"):
        normalize_mode(value)


def test_create_mission_payload_trims_text_and_generates_id():
    payload = create_mission_payload("  인명을 우선 확인해.  ")
    assert payload["text"] == "인명을 우선 확인해."
    assert payload["mission_id"].startswith("mission_ui_")


def test_empty_mission_is_rejected():
    with pytest.raises(ValueError, match="비어"):
        create_mission_payload("   ")


def test_rule_based_command_is_limited_to_start_stop():
    assert normalize_rule_based_command(" start ") == "START"
    assert normalize_rule_based_command("STOP") == "STOP"
    with pytest.raises(ValueError, match="START 또는 STOP"):
        normalize_rule_based_command("인명을 찾아")


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
    store.update({
        "schema_version": 1,
        "mode": "RULE_BASED",
        "timestamp": "2026-08-20T00:00:00+00:00",
        "mission": {
            "state": "EXPLORING",
            "target_type": "frontier",
            "current_target": {"frame_id": "map", "x": 1.2, "y": -0.3},
            "last_command": {
                "mission_id": "mission_ui_rule",
                "command": "START",
                "status": "ACCEPTED",
            },
        },
        "robot": {"battery_raw": 7200, "navigation_status": "RUNNING"},
        "exploration": {"status": "RUNNING"},
        "detections": {
            "targets": [{"type": "person_unconfirmed", "x": 0.5, "y": 0.0}],
            "counts": {"person": 1, "fire": 0},
        },
        "suppression": {"status": "IDLE"},
        "blocked_reason": "",
    }, "RULE_BASED")
    missions = []

    def submit(text, mode="VLA"):
        payload = create_mission_payload(text)
        if mode == "RULE_BASED":
            payload["mode"] = mode
        missions.append(payload)
        return payload

    owner = ControlModeOwner(lambda _mode: None)
    owner.select("VLA")
    server = FirefighterHTTPServer(
        "127.0.0.1", 0, store, submit, control_owner=owner
    )
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


def select_mode(server, mode):
    return request(
        server, "/api/control-mode", body=json.dumps({"mode": mode})
    )


def test_http_root_serves_required_v2_panels(http_server):
    server, _ = http_server
    status, content_type, body = request(server, "/")
    html = body.decode("utf-8")
    assert status == 200 and content_type.startswith("text/html")
    for text in (
        "Mission", "Robot Status", "Current Action", "Live Vision",
        "Semantic Map", "Situation Timeline", "Detected Objects",
        "Recent Result", "VLA Decision / Reason", "CONNECTED", "DISCONNECTED",
        "VLA Brain", "Rule-based", "modeSelector",
        'data-rule-command="START"', 'data-rule-command="STOP"',
    ):
        assert text in html
    for excluded in ("cmd_vel", "NavigateToPose", "Pump", "motor control"):
        assert excluded not in html
    for switching_guard in (
        "requestedMode!==activeMode",
        "render(rule?{}:{world_model:{}})",
        "$('missionForm').hidden=rule",
        "$('ruleControls').hidden=!rule",
    ):
        assert switching_guard in html
    for canonical_field in (
        "world.people", "world.fires", "world.current_action",
        "world.last_action", "decision.reason", "current.target_pose",
        "item.reported", "item.threatens_person", "item.threatened_person_id",
    ):
        assert canonical_field in html
    assert "보고 완료" in html
    assert "미보고" in html
    assert "사람 위협 화재" in html
    for timeline_guard in (
        "let lastRuleSituation=null",
        "mission.state!=='UNKNOWN'",
        "situation!==lastRuleSituation",
        "lastRuleSituation=null;renderTimeline()",
    ):
        assert timeline_guard in html


def test_submission_replay_fixture_is_explicit_and_renderable(http_server):
    fixture_path = (
        Path(__file__).parent / "fixtures" /
        "firefighter_ui_submission_replay.json"
    )
    payload = json.loads(fixture_path.read_text(encoding="utf-8"))
    presentation = payload["presentation"]

    assert presentation["mock_replay"] is True
    assert presentation["recorded_video"] is True
    assert payload["world_model"]["mission"]["text"] == (
        "Assess the situation and prioritize human safety."
    )
    assert [item["id"] for item in payload["world_model"]["people"]] == [
        "person_1"
    ]
    assert [item["id"] for item in payload["world_model"]["fires"]] == [
        "fire_1", "fire_2"
    ]
    assert len(presentation["timeline"]) == 5
    assert presentation["result"]["title"] == "SCENE ASSESSED"

    server, _ = http_server
    status, _, body = request(server, "/")
    html = body.decode("utf-8")
    assert status == 200
    for marker in (
        "replayBadge", "visionVideo", "generatedPlan", "resultChecklist",
        "renderReplayTimeline", "presentation.relationships",
        "VLA PLAN GENERATED",
    ):
        assert marker in html

    status, content_type, body = request(
        server, "/media/fire_person_detection_result.mp4"
    )
    assert status == 200
    assert content_type == "video/mp4"
    assert body[4:8] == b"ftyp"


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


def test_status_api_selects_rule_based_snapshot(http_server):
    server, _ = http_server
    status, _, body = request(server, "/api/status?mode=RULE_BASED")
    payload = json.loads(body)
    assert status == 200
    assert payload["mode"] == "RULE_BASED"
    assert payload["mission"]["state"] == "EXPLORING"
    assert payload["robot"]["navigation_status"] == "RUNNING"
    assert payload["detections"]["counts"] == {"person": 1, "fire": 0}


def test_status_api_switches_vla_rule_vla_without_cross_mode_data(http_server):
    server, _ = http_server
    _, _, first_vla_body = request(server, "/api/status?mode=VLA")
    _, _, rule_body = request(server, "/api/status?mode=RULE_BASED")
    _, _, second_vla_body = request(server, "/api/status?mode=VLA")

    first_vla = json.loads(first_vla_body)
    rule = json.loads(rule_body)
    second_vla = json.loads(second_vla_body)
    assert first_vla == second_vla
    assert "world_model" in first_vla and "mission" not in first_vla
    assert rule["mode"] == "RULE_BASED"
    assert "mission" in rule and "world_model" not in rule


def test_status_api_rejects_unknown_mode(http_server):
    server, _ = http_server
    with pytest.raises(HTTPError) as error:
        request(server, "/api/status?mode=UNKNOWN")
    assert error.value.code == 400


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


def test_mission_api_routes_rule_based_mode(http_server):
    server, missions = http_server
    select_mode(server, "RULE_BASED")
    status, _, body = request(
        server,
        "/api/mission",
        body=json.dumps({"text": " start ", "mode": "RULE_BASED"}),
    )
    response = json.loads(body)
    assert status == 202
    assert len(missions) == 1
    assert response["text"] == "START"
    assert response["mode"] == "RULE_BASED"


def test_mission_api_cross_routing_counts_are_isolated(http_server):
    server, missions = http_server
    request(
        server, "/api/mission",
        body=json.dumps({"text": "인명을 찾아", "mode": "VLA"}),
    )
    with pytest.raises(HTTPError) as error:
        request(
            server, "/api/mission",
            body=json.dumps({"text": "START", "mode": "RULE_BASED"}),
        )
    assert error.value.code == 400

    vla = [mission for mission in missions if mission.get("mode") is None]
    rule = [mission for mission in missions if mission.get("mode") == "RULE_BASED"]
    assert len(vla) == 1
    assert rule == []


def test_mission_api_rejects_rule_based_free_text(http_server):
    server, missions = http_server
    select_mode(server, "RULE_BASED")
    with pytest.raises(HTTPError) as error:
        request(
            server,
            "/api/mission",
            body=json.dumps({"text": "인명을 찾아", "mode": "RULE_BASED"}),
        )
    assert error.value.code == 400
    assert missions == []


def test_control_mode_is_shared_by_multiple_clients(http_server):
    server, _ = http_server
    select_mode(server, "RULE_BASED")
    _, _, first = request(server, "/api/control-mode")
    _, _, second = request(server, "/api/control-mode")
    assert json.loads(first) == json.loads(second)
    assert json.loads(first)["active_control_mode"] == "RULE_BASED"


def test_active_mission_blocks_mode_change(http_server):
    server, _ = http_server
    request(server, "/api/mission", body=json.dumps({"text": "화재를 진압해"}))
    with pytest.raises(HTTPError) as error:
        select_mode(server, "RULE_BASED")
    assert error.value.code == 409
    assert "MODE_CHANGE_BLOCKED_ACTIVE_ACTION" in error.value.read().decode()


def test_terminal_and_stop_allow_mode_change(http_server):
    server, _ = http_server
    request(server, "/api/mission", body=json.dumps({"text": "화재를 진압해"}))
    server._control_owner.observe_terminal(
        "VLA", {"world_model": {"mission": {"status": "COMPLETED"}, "current_action": None}}
    )
    select_mode(server, "RULE_BASED")
    request(
        server, "/api/mission",
        body=json.dumps({"text": "STOP", "mode": "RULE_BASED"}),
    )
    select_mode(server, "VLA")


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
        "mission_scope": "FULL_EXPLORATION",
        "action": "EXTINGUISH",
        "target": "fire_0001",
        "reason": "화재 진압을 수행합니다.",
    }


def test_existing_launches_do_not_force_start_ui():
    from pathlib import Path

    launch_dir = Path(__file__).parents[2] / "fire_vla_bringup" / "launch"
    for name in ("local_mock_vla.launch.py", "topic_bridge_vla.launch.py"):
        assert "firefighter_ui" not in (launch_dir / name).read_text()

def test_ui_software_e2e_completes_scoped_person_fire_mission():
    world = WorldModel(WorldModelConfig())
    world.update_robot_pose(Pose2D(0.0, 0.0))
    store = StatusStore()
    submitted = []

    def submit(text):
        payload = create_mission_payload(text)
        submitted.append(payload)
        world.set_mission(payload["mission_id"], payload["text"])
        return payload

    owner = ControlModeOwner(lambda _mode: None)
    owner.select("VLA")
    server = FirefighterHTTPServer(
        "127.0.0.1", 0, store, submit, control_owner=owner
    )
    server.start()
    try:
        status, _, _ = request(
            server,
            "/api/mission",
            body=json.dumps({"text": "사람을 위협하는 화재를 진압해줘"}),
        )
        assert status == 202
        now = utc_now_iso()
        world.update_observation_batch(ObservationBatch(now, (
            SemanticObservation(
                "person_01", "person", .95, Pose2D(2.05, 0.0), now
            ),
            SemanticObservation(
                "fire_01", "fire", .92, Pose2D(2.0, 0.0), now, "SMALL"
            ),
        )))

        publisher = MagicMock()
        node = MagicMock()
        node.create_publisher.return_value = publisher
        report = TopicBridgePersonReportAdapter(node, world)
        assert report.publish_new_people() == 1
        assert publisher.publish.call_count == 1

        tracker = VLAStatusTracker()
        active_payload = tracker.create_payload(world.create_snapshot())
        store.update(active_payload)
        _, _, active_body = request(server, "/api/status")
        active = json.loads(active_body)
        active_person = active["world_model"]["people"][0]
        active_fire = active["world_model"]["fires"][0]
        assert active_person["reported"] is True
        assert active_fire["state"] == "ACTIVE"
        assert active_fire["threatens_person"] is True
        assert active_fire["threatened_person_id"] == "person_01"
        _, _, html_body = request(server, "/")
        html = html_body.decode("utf-8")
        assert "보고 완료" in html
        assert "사람 위협 화재" in html
        assert "item.threatened_person_id" in html

        class OneDecision:
            calls = 0

            def decide(self, mission, snapshot):
                self.calls += 1
                return ActionDecision(
                    ActionType.NAVIGATE_TO,
                    "위협 화점으로 접근",
                    "fire_01",
                    MissionScope.PERSON_FIRE,
                )

        results = MockResultQueue()
        navigation = MockNavigationAdapter(results)
        spray = MockSprayAdapter(results)
        brain = OneDecision()
        orchestrator = VLAOrchestrator(
            world,
            brain,
            TargetResolver(),
            ActionValidator(),
            ActionDispatcher(
                navigation, spray, report, MockWaitAdapter(results)
            ),
        )
        nav_cycle = orchestrator.decide_once()
        assert nav_cycle.decision.action == ActionType.NAVIGATE_TO
        world.update_robot_pose(Pose2D(1.75, 0.0))
        refreshed = utc_now_iso()
        world.update_observation_batch(ObservationBatch(refreshed, (
            SemanticObservation(
                "person_01", "person", .95, Pose2D(2.05, 0.0), refreshed
            ),
            SemanticObservation(
                "fire_01", "fire", .92, Pose2D(2.0, 0.0), refreshed, "SMALL"
            ),
        )))
        assert orchestrator.process_results(results) == 1

        spray_cycle = orchestrator.decide_once()
        assert spray_cycle.decision.action == ActionType.EXTINGUISH
        assert orchestrator.process_results(results) == 1
        assert world.fires["fire_01"].state == FireState.PENDING_VERIFICATION

        time.sleep(world.config.verification_delay_sec + 0.05)
        for _ in range(world.config.verification_required_observations):
            world.update_observation_batch(
                ObservationBatch(utc_now_iso(), tuple())
            )

        final_payload = tracker.create_payload(world.create_snapshot())
        store.update(final_payload)
        _, _, final_body = request(server, "/api/status")
        final = json.loads(final_body)["world_model"]
        assert world.fires["fire_01"].state == FireState.EXTINGUISHED
        assert final["mission"]["status"] == "COMPLETED"
        assert "world.mission?.status" in html
        assert brain.calls == 1
        assert len(submitted) == 1
        assert len(navigation.calls) == 1
        assert len(spray.calls) == 1
        assert publisher.publish.call_count == 1
    finally:
        server.close()
