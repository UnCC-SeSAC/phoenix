import json
from datetime import datetime

import pytest

from fire_vla_core.adapters.mock_adapters import (
    MockNavigationAdapter,
    MockReportAdapter,
    MockResultQueue,
    MockWaitAdapter,
)
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import (
    ActionDecision,
    ActionSubmissionStatus,
    ActionType,
    FireEntity,
    FireState,
    Pose2D,
    utc_now_iso,
)
from fire_vla_core.orchestrator import VLAOrchestrator
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.ros.perception_normalizer import CanonicalPerceptionNormalizer
from fire_vla_core.ros.topic_bridge_spray_adapter import TopicBridgeSprayAdapter
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


def make_system(
    *,
    target="fire_0001",
    state=FireState.ACTIVE,
    in_range=True,
    spray_count=0,
    llm=None,
):
    world = WorldModel()
    world.set_mission("mission_01", "화점을 진압해")
    world.fires["fire_0001"] = FireEntity(
        "fire_0001",
        Pose2D(0.5 if in_range else 2.0, 0.0),
        confidence=0.94,
        state=state,
        spray_count=spray_count,
    )
    world.update_robot_pose(Pose2D(0.0, 0.0))
    node = FakeNode()
    spray = TopicBridgeSprayAdapter(node, world)
    results = MockResultQueue()
    brain = llm or SequenceLLM(
        ActionDecision(ActionType.EXTINGUISH, "화점 분사", target)
    )
    orchestrator = VLAOrchestrator(
        world,
        brain,
        TargetResolver(),
        ActionValidator(),
        ActionDispatcher(
            MockNavigationAdapter(results),
            spray,
            MockReportAdapter(results),
            MockWaitAdapter(results),
        ),
    )
    return world, node, spray, orchestrator, brain


def command_payload(node):
    messages = node.publishers["/vla/spray_command"].messages
    assert len(messages) == 1
    return json.loads(messages[0].data)


def emit_result(node, action_id, status, fire_id="fire_0001"):
    message = type("Message", (), {})()
    message.data = json.dumps(
        {
            "action_id": action_id,
            "status": status,
            "fire_id": fire_id,
            "message": "spray consumer result",
        }
    )
    node.subscriptions["/vla/spray_result"](message)


def test_valid_fire_publishes_canonical_command_from_world_model():
    world, node, _, orchestrator, _ = make_system()

    cycle = orchestrator.decide_once()

    assert cycle.validation.approved is True
    assert cycle.submission.status == ActionSubmissionStatus.ACCEPTED
    assert world.current_action is cycle.validation.action
    assert world.fires["fire_0001"].state == FireState.ACTIVE
    assert world.fires["fire_0001"].spray_count == 0
    payload = command_payload(node)
    assert payload["action_id"] == cycle.validation.action.action_id
    assert payload["mission_id"] == "mission_01"
    assert payload["fire_id"] == "fire_0001"
    assert payload["command"] == "SPRAY"
    assert "reason" not in payload
    assert "fire_position" not in payload
    datetime.fromisoformat(payload["timestamp"])


def test_success_enters_pending_verification_not_extinguished():
    world, node, spray, orchestrator, _ = make_system()
    action = orchestrator.decide_once().validation.action

    emit_result(node, action.action_id, "SUCCEEDED")
    assert orchestrator.process_results(spray) == 1

    fire = world.fires["fire_0001"]
    assert fire.spray_count == 1
    assert fire.state == FireState.PENDING_VERIFICATION
    assert fire.verification_started_at is not None
    assert world.current_action is None
    assert world.last_action is action
    assert world.last_action.status.value == "SUCCEEDED"


@pytest.mark.parametrize(
    "status",
    ["FAILED", "ABORTED", "CANCELED", "TIMED_OUT"],
)
def test_non_success_terminal_result_keeps_fire_active(status):
    world, node, spray, orchestrator, _ = make_system()
    action = orchestrator.decide_once().validation.action

    emit_result(node, action.action_id, status)
    assert orchestrator.process_results(spray) == 1

    fire = world.fires["fire_0001"]
    assert fire.state == FireState.ACTIVE
    assert fire.spray_count == 0
    assert world.current_action is None
    assert world.last_action.status.value == status


def test_out_of_range_fire_is_rejected_without_publish():
    _, node, _, orchestrator, _ = make_system(in_range=False)
    cycle = orchestrator.decide_once()
    assert cycle.submission is None
    assert "분사 가능 범위" in cycle.validation.reason
    assert node.publishers["/vla/spray_command"].messages == []


@pytest.mark.parametrize(
    "state",
    [FireState.PENDING_VERIFICATION, FireState.EXTINGUISHED, FireState.INACCESSIBLE],
)
def test_inactive_fire_is_rejected_without_publish(state):
    _, node, _, orchestrator, _ = make_system(state=state)
    cycle = orchestrator.decide_once()
    assert cycle.submission is None
    assert "ACTIVE" in cycle.validation.reason
    assert node.publishers["/vla/spray_command"].messages == []


def test_max_attempts_is_rejected_without_publish():
    _, node, _, orchestrator, _ = make_system(spray_count=2)
    cycle = orchestrator.decide_once()
    assert cycle.submission is None
    assert "최대 분사" in cycle.validation.reason
    assert node.publishers["/vla/spray_command"].messages == []


def test_unknown_fire_is_rejected_without_publish():
    _, node, _, orchestrator, _ = make_system(target="fire_9999")
    cycle = orchestrator.decide_once()
    assert cycle.submission is None
    assert "WorldModel" in cycle.validation.reason
    assert node.publishers["/vla/spray_command"].messages == []


def test_dispatcher_and_adapter_prevent_duplicate_publish():
    _, node, spray, orchestrator, _ = make_system()
    action = orchestrator.decide_once().validation.action
    assert orchestrator.dispatcher.submit(action).status == ActionSubmissionStatus.DUPLICATE
    assert spray.submit(action).status == ActionSubmissionStatus.DUPLICATE
    assert len(node.publishers["/vla/spray_command"].messages) == 1


def test_stop_publishes_correlated_cancel_and_waits_for_result():
    world, node, spray, orchestrator, _ = make_system()
    action = orchestrator.decide_once().validation.action
    assert spray.stop() is True
    cancel = json.loads(node.publishers["/vla/spray_cancel"].messages[0].data)
    assert cancel == {"action_id": action.action_id}
    assert world.current_action is action

    emit_result(node, action.action_id, "CANCELED")
    assert orchestrator.process_results(spray) == 1
    assert world.current_action is None


def test_stale_result_does_not_clear_or_complete_active_spray():
    world, node, spray, orchestrator, _ = make_system()
    action = orchestrator.decide_once().validation.action
    emit_result(node, "action_stale", "SUCCEEDED")
    assert orchestrator.process_results(spray) == 0
    assert world.current_action is action
    assert world.fires["fire_0001"].spray_count == 0


def test_mismatched_fire_result_is_ignored():
    world, node, spray, orchestrator, _ = make_system()
    action = orchestrator.decide_once().validation.action
    emit_result(node, action.action_id, "SUCCEEDED", "fire_other")
    assert orchestrator.process_results(spray) == 0
    assert world.current_action is action
    assert world.fires["fire_0001"].spray_count == 0
    assert node.logger.warnings


def test_duplicate_terminal_result_is_applied_once():
    world, node, spray, orchestrator, _ = make_system()
    action = orchestrator.decide_once().validation.action
    emit_result(node, action.action_id, "SUCCEEDED")
    emit_result(node, action.action_id, "SUCCEEDED")
    assert orchestrator.process_results(spray) == 1
    assert world.fires["fire_0001"].spray_count == 1
    assert len([
        event for event in world.event_log
        if event.event_type == "SPRAY_SUCCEEDED"
    ]) == 1


def test_physical_result_invalidates_signature_for_next_decision():
    llm = SequenceLLM(
        ActionDecision(ActionType.EXTINGUISH, "화점 분사", "fire_0001"),
        ActionDecision(ActionType.WAIT, "결과 이후 대기", None),
    )
    world, node, spray, orchestrator, brain = make_system(llm=llm)
    action = orchestrator.decide_once().validation.action
    emit_result(node, action.action_id, "FAILED")
    orchestrator.process_results(spray)

    next_cycle = orchestrator.decide_once()

    assert brain.calls == 2
    assert world.current_action is None
    assert next_cycle.decision.action == ActionType.WAIT


def test_idless_vla03a_fire_flows_through_spray_success():
    world = WorldModel()
    world.set_mission("mission_03a", "화점을 진압해")
    world.update_robot_pose(Pose2D(0.0, 0.0))
    normalizer = CanonicalPerceptionNormalizer(world)
    world.update_observation_batch(normalizer.normalize({
        "timestamp": utc_now_iso(),
        "frame_id": "map",
        "detections": [{
            "class_name": "fire",
            "confidence": 0.9,
            "map_position": {"x": 0.5, "y": 0.0},
        }],
    }))
    assert list(world.fires) == ["fire_0001"]
    assert world.fires["fire_0001"].robot_within_spray_range is True

    node = FakeNode()
    spray = TopicBridgeSprayAdapter(node, world)
    results = MockResultQueue()
    orchestrator = VLAOrchestrator(
        world,
        SequenceLLM(ActionDecision(
            ActionType.EXTINGUISH, "stable fire 진압", "fire_0001"
        )),
        TargetResolver(),
        ActionValidator(),
        ActionDispatcher(
            MockNavigationAdapter(results), spray,
            MockReportAdapter(results), MockWaitAdapter(results),
        ),
    )
    action = orchestrator.decide_once().validation.action
    assert command_payload(node)["fire_id"] == "fire_0001"
    emit_result(node, action.action_id, "SUCCEEDED")
    assert orchestrator.process_results(spray) == 1
    assert world.fires["fire_0001"].state == FireState.PENDING_VERIFICATION


def test_invalid_result_is_ignored():
    _, node, spray, orchestrator, _ = make_system()
    orchestrator.decide_once()
    message = type("Message", (), {})()
    message.data = '{"action_id":"action_0001","status":"NOT_A_STATUS"}'
    node.subscriptions["/vla/spray_result"](message)
    assert orchestrator.process_results(spray) == 0
    assert node.logger.warnings
