import json
from datetime import timedelta

import pytest

from fire_vla_core.adapters.mock_adapters import (
    MockReportAdapter,
    MockResultQueue,
    MockSprayAdapter,
    MockWaitAdapter,
)
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import (
    Action,
    ActionDecision,
    ActionSubmissionStatus,
    ActionType,
    FireEntity,
    PersonEntity,
    Pose2D,
    utc_now,
)
from fire_vla_core.orchestrator import VLAOrchestrator
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.ros.topic_bridge_navigation_adapter import (
    TopicBridgeNavigationAdapter,
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


class StubLLM:
    def __init__(self, decision):
        self.decision = decision

    def decide(self, mission, world_model):
        return self.decision


def make_system(decision):
    world = WorldModel()
    world.update_robot_pose(Pose2D(0.0, 0.0))
    world.set_mission("mission_01", "대상으로 이동")
    world.people["person_01"] = PersonEntity(
        "person_01", Pose2D(2.0, 1.0, 0.7)
    )
    world.fires["fire_01"] = FireEntity(
        "fire_01", Pose2D(3.0, -1.0, -0.5)
    )
    node = FakeNode()
    navigation = TopicBridgeNavigationAdapter(node)
    results = MockResultQueue()
    dispatcher = ActionDispatcher(
        navigation,
        MockSprayAdapter(results),
        MockReportAdapter(results),
        MockWaitAdapter(results),
    )
    orchestrator = VLAOrchestrator(
        world,
        StubLLM(decision),
        TargetResolver(),
        ActionValidator(),
        dispatcher,
    )
    return world, node, navigation, dispatcher, orchestrator


def goal_payload(node):
    messages = node.publishers["/vla/navigation_goal"].messages
    assert len(messages) == 1
    return json.loads(messages[0].data)


@pytest.mark.parametrize(
    ("target_id", "expected_x", "expected_y", "expected_yaw"),
    [
        ("person_01", 2.0, 1.0, pytest.approx(0.463647609)),
        (
            "fire_01",
            pytest.approx(2.810263340),
            pytest.approx(-0.936754447),
            pytest.approx(-0.321750554),
        ),
    ],
)
def test_navigate_target_is_resolved_and_published(
    target_id, expected_x, expected_y, expected_yaw
):
    decision = ActionDecision(ActionType.NAVIGATE_TO, "대상으로 이동", target_id)
    world, node, _, _, orchestrator = make_system(decision)

    cycle = orchestrator.decide_once()

    assert cycle.validation.approved is True
    assert cycle.submission.status == ActionSubmissionStatus.ACCEPTED
    assert world.current_action is cycle.validation.action
    payload = goal_payload(node)
    assert payload == {
        "action_id": cycle.validation.action.action_id,
        "action": "NAVIGATE_TO",
        "target_id": target_id,
        "target_pose": {
            "x": expected_x,
            "y": expected_y,
            "yaw": expected_yaw,
        },
        "frame_id": "map",
    }


def test_missing_target_does_not_publish_navigation_goal():
    decision = ActionDecision(
        ActionType.NAVIGATE_TO, "없는 대상으로 이동", "person_999"
    )
    _, node, _, _, orchestrator = make_system(decision)

    cycle = orchestrator.decide_once()

    assert cycle.submission is None
    assert cycle.blocked_reason.startswith("TARGET_RESOLUTION_FAILED:")
    assert node.publishers["/vla/navigation_goal"].messages == []


def test_stale_robot_pose_validation_reject_does_not_publish():
    decision = ActionDecision(
        ActionType.NAVIGATE_TO, "사람에게 이동", "person_01"
    )
    world, node, _, _, orchestrator = make_system(decision)
    world.robot.pose_updated_at = (utc_now() - timedelta(seconds=10)).isoformat()

    cycle = orchestrator.decide_once()

    assert cycle.submission is None
    assert cycle.blocked_reason.startswith("ACTION_VALIDATION_REJECTED:")
    assert "오래" in cycle.validation.reason
    assert node.publishers["/vla/navigation_goal"].messages == []


def test_dispatcher_and_adapter_prevent_duplicate_goal_publish():
    decision = ActionDecision(
        ActionType.NAVIGATE_TO, "사람에게 이동", "person_01"
    )
    _, node, navigation, dispatcher, orchestrator = make_system(decision)
    cycle = orchestrator.decide_once()
    action = cycle.validation.action

    assert dispatcher.submit(action).status == ActionSubmissionStatus.DUPLICATE
    assert navigation.submit(action).status == ActionSubmissionStatus.DUPLICATE
    assert len(node.publishers["/vla/navigation_goal"].messages) == 1


def test_cancel_and_correlated_result_preserve_adapter_state():
    decision = ActionDecision(
        ActionType.NAVIGATE_TO, "사람에게 이동", "person_01"
    )
    _, node, navigation, _, orchestrator = make_system(decision)
    cycle = orchestrator.decide_once()
    action_id = cycle.validation.action.action_id

    assert navigation.cancel_current() is True
    cancel = json.loads(node.publishers["/vla/navigation_cancel"].messages[0].data)
    assert cancel == {"action_id": action_id}

    result_message = type("Message", (), {})()
    result_message.data = json.dumps(
        {
            "action_id": action_id,
            "status": "CANCELED",
            "target_id": "person_01",
            "message": "canceled for test",
        }
    )
    node.subscriptions["/vla/navigation_result"](result_message)
    results = navigation.drain_results()

    assert len(results) == 1
    assert results[0].action_id == action_id
    assert results[0].status.value == "CANCELED"
    next_action = Action(
        "action_next",
        ActionType.NAVIGATE_TO,
        "다음 이동",
        target="fire_01",
        target_pose=Pose2D(3.0, -1.0),
    )
    assert navigation.submit(next_action).status == ActionSubmissionStatus.ACCEPTED
    assert len(node.publishers["/vla/navigation_goal"].messages) == 2
