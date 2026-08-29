import json
from datetime import datetime, timedelta

import pytest

from fire_vla_core.adapters.mock_adapters import (
    MockNavigationAdapter,
    MockResultQueue,
    MockSprayAdapter,
    MockWaitAdapter,
)
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import (
    ActionDecision,
    ActionResultStatus,
    ActionSubmissionStatus,
    ActionType,
    PersonEntity,
    PersonState,
    Pose2D,
    utc_now,
    utc_now_iso,
)
from fire_vla_core.orchestrator import VLAOrchestrator
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.ros.perception_normalizer import CanonicalPerceptionNormalizer
from fire_vla_core.ros.topic_bridge_person_report_adapter import (
    TopicBridgePersonReportAdapter,
)
from fire_vla_core.validator import ActionValidator
from fire_vla_core.world_model import WorldModel


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


class FakeNode:
    def __init__(self):
        self.publishers = {}
        self.subscriptions = {}
        self.logger = FakeLogger()

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return callback

    def get_logger(self):
        return self.logger


class SequenceLLM:
    def __init__(self, *decisions):
        self.decisions = list(decisions)
        self.calls = 0

    def decide(self, mission, world_model):
        decision = self.decisions[min(self.calls, len(self.decisions) - 1)]
        self.calls += 1
        return decision


def make_system(*, reported=False, target="person_0001", llm=None):
    world = WorldModel()
    world.update_robot_pose(Pose2D(0.0, 0.0))
    world.set_mission("mission_01", "발견한 사람을 보고해")
    world.people["person_0001"] = PersonEntity(
        "person_0001",
        Pose2D(2.0, 1.0, 0.5),
        confidence=0.91,
        reported=reported,
        state=PersonState.REPORTED if reported else PersonState.DETECTED,
    )
    node = FakeNode()
    report = TopicBridgePersonReportAdapter(node, world)
    results = MockResultQueue()
    dispatcher = ActionDispatcher(
        MockNavigationAdapter(results),
        MockSprayAdapter(results),
        report,
        MockWaitAdapter(results),
    )
    brain = llm or SequenceLLM(
        ActionDecision(ActionType.REPORT_PERSON, "사람을 보고", target)
    )
    orchestrator = VLAOrchestrator(
        world, brain, TargetResolver(), ActionValidator(), dispatcher
    )
    return world, node, report, dispatcher, orchestrator, brain


def report_payload(node):
    messages = node.publishers["/vla/person_report"].messages
    assert len(messages) == 1
    return json.loads(messages[0].data)


def emit_result(node, action_id, status, person_id="person_0001"):
    message = type("Message", (), {})()
    message.data = json.dumps(
        {
            "action_id": action_id,
            "status": status,
            "person_id": person_id,
            "message": "report consumer result",
        }
    )
    node.subscriptions["/vla/person_report_result"](message)


def test_report_person_publishes_authoritative_world_model_payload():
    world, node, _, _, orchestrator, _ = make_system()

    cycle = orchestrator.decide_once()

    assert cycle.submission.status == ActionSubmissionStatus.ACCEPTED
    assert world.people["person_0001"].reported is False
    assert cycle.validation.action.action_id in world.pending_actions
    payload = report_payload(node)
    assert payload["action_id"] == cycle.validation.action.action_id
    assert payload["mission_id"] == "mission_01"
    assert payload["person_id"] == "person_0001"
    assert payload["map_position"] == {"x": 2.0, "y": 1.0}
    assert payload["confidence"] == 0.91
    assert payload["frame_id"] == "map"
    assert "reason" not in payload
    datetime.fromisoformat(payload["timestamp"])


def test_new_person_is_auto_reported_once_without_ack_or_pending_action():
    world, node, report, _, _, _ = make_system()

    assert report.publish_new_people() == 1
    payload = report_payload(node)
    assert payload["mission_id"] == "mission_01"
    assert payload["person_id"] == "person_0001"
    assert payload["map_position"] == {"x": 2.0, "y": 1.0}
    assert payload["confidence"] == 0.91
    assert payload["frame_id"] == "map"
    assert world.people["person_0001"].reported is True
    assert world.people["person_0001"].state == PersonState.REPORTED
    assert world.current_action is None
    assert world.pending_actions == {}
    assert report.publish_new_people() == 0
    assert len(node.publishers["/vla/person_report"].messages) == 1


def test_same_person_is_auto_reported_once_per_mission():
    world, node, report, _, _, _ = make_system()
    assert report.publish_new_people() == 1

    world.set_mission("mission_02", "새 임무")
    world.people["person_0001"] = PersonEntity(
        "person_0001", Pose2D(2.0, 1.0), confidence=0.91
    )

    assert report.publish_new_people() == 1
    assert report.publish_new_people() == 0
    assert len(node.publishers["/vla/person_report"].messages) == 2


def test_idless_reported_person_after_ttl_is_not_reported_again():
    world = WorldModel()
    world.set_mission("mission_01", "사람 위치를 보고해")
    node = FakeNode()
    report = TopicBridgePersonReportAdapter(node, world)
    normalizer = CanonicalPerceptionNormalizer(world)

    first = {
        "timestamp": utc_now().isoformat(),
        "frame_id": "map",
        "detections": [{
            "class_name": "person",
            "confidence": 0.91,
            "map_position": {"x": 2.0, "y": 1.0},
        }],
    }
    world.update_observation_batch(normalizer.normalize(first))
    assert report.publish_new_people() == 1
    assert world.people["person_0001"].reported is True
    world.people["person_0001"].last_seen = (
        utc_now() - timedelta(seconds=3)
    ).isoformat()

    second = {
        "timestamp": utc_now().isoformat(),
        "frame_id": "map",
        "detections": [{
            "class_name": "person",
            "confidence": 0.91,
            "map_position": {"x": 2.004, "y": 1.0},
        }],
    }
    world.update_observation_batch(normalizer.normalize(second))

    assert list(world.people) == ["person_0001"]
    assert report.publish_new_people() == 0
    assert len(node.publishers["/vla/person_report"].messages) == 1


def test_success_result_marks_person_reported_only_after_terminal_result():
    world, node, report, _, orchestrator, _ = make_system()
    cycle = orchestrator.decide_once()
    action = cycle.validation.action
    assert world.people[action.target].reported is False

    emit_result(node, action.action_id, "SUCCEEDED")
    assert orchestrator.process_results(report) == 1

    person = world.people["person_0001"]
    assert person.reported is True
    assert person.state == PersonState.REPORTED
    assert action.action_id not in world.pending_actions
    assert world.last_action is action
    assert world.last_action.status.value == "SUCCEEDED"


@pytest.mark.parametrize(
    "status",
    ["FAILED", "ABORTED", "CANCELED", "TIMED_OUT"],
)
def test_non_success_terminal_result_does_not_mark_reported(status):
    world, node, report, _, orchestrator, _ = make_system()
    cycle = orchestrator.decide_once()
    action = cycle.validation.action

    emit_result(node, action.action_id, status)
    assert orchestrator.process_results(report) == 1

    assert world.people["person_0001"].reported is False
    assert world.people["person_0001"].state == PersonState.DETECTED
    assert action.action_id not in world.pending_actions
    assert world.last_action.status.value == status


def test_reported_person_is_rejected_without_publish():
    _, node, _, _, orchestrator, _ = make_system(reported=True)

    cycle = orchestrator.decide_once()

    assert cycle.submission is None
    assert cycle.validation.approved is False
    assert "이미 보고" in cycle.validation.reason
    assert node.publishers["/vla/person_report"].messages == []


def test_unknown_person_is_rejected_without_publish():
    _, node, _, _, orchestrator, _ = make_system(target="person_9999")

    cycle = orchestrator.decide_once()

    assert cycle.submission is None
    assert cycle.validation.approved is False
    assert node.publishers["/vla/person_report"].messages == []


def test_dispatcher_and_adapter_prevent_duplicate_report_publish():
    _, node, report, dispatcher, orchestrator, _ = make_system()
    cycle = orchestrator.decide_once()
    action = cycle.validation.action

    assert dispatcher.submit(action).status == ActionSubmissionStatus.DUPLICATE
    assert report.submit(action).status == ActionSubmissionStatus.DUPLICATE
    assert len(node.publishers["/vla/person_report"].messages) == 1


def test_stale_result_does_not_complete_pending_report():
    world, node, report, _, orchestrator, _ = make_system()
    cycle = orchestrator.decide_once()
    action = cycle.validation.action

    emit_result(node, "action_stale", "SUCCEEDED")

    assert orchestrator.process_results(report) == 0
    assert action.action_id in world.pending_actions
    assert world.people["person_0001"].reported is False
    assert world.last_action is None


def test_mismatched_person_result_is_ignored():
    world, node, report, _, orchestrator, _ = make_system()
    action = orchestrator.decide_once().validation.action

    emit_result(node, action.action_id, "SUCCEEDED", "person_other")

    assert orchestrator.process_results(report) == 0
    assert world.people["person_0001"].reported is False
    assert action.action_id in world.pending_actions
    assert node.logger.warnings


def test_duplicate_terminal_result_is_applied_once():
    world, node, report, _, orchestrator, _ = make_system()
    action = orchestrator.decide_once().validation.action
    emit_result(node, action.action_id, "SUCCEEDED")
    emit_result(node, action.action_id, "SUCCEEDED")

    assert orchestrator.process_results(report) == 1
    terminal_events = [
        event for event in world.event_log
        if event.event_type == "REPORT_SUCCEEDED"
    ]
    assert len(terminal_events) == 1


def test_success_changes_signature_and_allows_next_decision():
    llm = SequenceLLM(
        ActionDecision(ActionType.REPORT_PERSON, "사람을 보고", "person_0001"),
        ActionDecision(ActionType.WAIT, "다음 지시 대기", None),
    )
    world, node, report, _, orchestrator, brain = make_system(llm=llm)
    action = orchestrator.decide_once().validation.action
    emit_result(node, action.action_id, "SUCCEEDED")
    orchestrator.process_results(report)

    next_cycle = orchestrator.decide_once()

    assert brain.calls == 2
    assert world.people["person_0001"].reported is True
    assert next_cycle.decision.action == ActionType.WAIT


def test_idless_vla03a_person_flows_through_report_success():
    world = WorldModel()
    world.update_robot_pose(Pose2D(0.0, 0.0))
    world.set_mission("mission_03a", "발견한 사람을 보고해")
    normalizer = CanonicalPerceptionNormalizer(world)
    payload = {
        "timestamp": utc_now_iso(),
        "frame_id": "map",
        "detections": [
            {
                "class_name": "person",
                "confidence": 0.88,
                "map_position": {"x": 1.2, "y": -0.4},
            }
        ],
    }
    world.update_observation_batch(normalizer.normalize(payload))
    assert list(world.people) == ["person_0001"]

    node = FakeNode()
    report = TopicBridgePersonReportAdapter(node, world)
    results = MockResultQueue()
    orchestrator = VLAOrchestrator(
        world,
        SequenceLLM(ActionDecision(
            ActionType.REPORT_PERSON, "stable ID person 보고", "person_0001"
        )),
        TargetResolver(),
        ActionValidator(),
        ActionDispatcher(
            MockNavigationAdapter(results),
            MockSprayAdapter(results),
            report,
            MockWaitAdapter(results),
        ),
    )
    action = orchestrator.decide_once().validation.action
    assert report_payload(node)["person_id"] == "person_0001"
    emit_result(node, action.action_id, "SUCCEEDED")
    assert orchestrator.process_results(report) == 1
    assert world.people["person_0001"].reported is True


def test_invalid_result_is_ignored():
    _, node, report, _, orchestrator, _ = make_system()
    orchestrator.decide_once()
    message = type("Message", (), {})()
    message.data = '{"action_id":"action_0001","status":"NOT_A_STATUS"}'

    node.subscriptions["/vla/person_report_result"](message)

    assert orchestrator.process_results(report) == 0
    assert node.logger.warnings
