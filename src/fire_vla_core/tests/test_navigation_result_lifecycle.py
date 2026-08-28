import json
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "uncc_example"))
nav2_msgs = ModuleType("nav2_msgs")
nav2_msgs_action = ModuleType("nav2_msgs.action")
nav2_msgs_action.ComputePathToPose = type("ComputePathToPose", (), {})
nav2_msgs_action.NavigateToPose = type("NavigateToPose", (), {})
nav2_msgs.action = nav2_msgs_action
sys.modules.setdefault("nav2_msgs", nav2_msgs)
sys.modules.setdefault("nav2_msgs.action", nav2_msgs_action)

import pytest
from action_msgs.msg import GoalStatus

from fire_vla_core.adapters.mock_adapters import (
    MockReportAdapter,
    MockResultQueue,
    MockSprayAdapter,
    MockWaitAdapter,
)
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import (
    ActionDecision,
    ActionLifecycleStatus,
    ActionType,
    PersonEntity,
    Pose2D,
)
from fire_vla_core.orchestrator import VLAOrchestrator
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.ros.topic_bridge_navigation_adapter import TopicBridgeNavigationAdapter
from fire_vla_core.validator import ActionValidator
from fire_vla_core.world_model import WorldModel
from uncc_example.vla_navigation_bridge_node import PendingGoal, VLANavigationBridgeNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeLogger:
    def warning(self, message):
        pass


class FakeNode:
    def __init__(self):
        self.publishers = {}
        self.subscriptions = {}

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        self.subscriptions[topic] = callback
        return callback

    def get_logger(self):
        return FakeLogger()


class StubLLM:
    def __init__(self):
        self.calls = 0

    def decide(self, mission, world_model):
        self.calls += 1
        return ActionDecision(ActionType.NAVIGATE_TO, "사람에게 이동", "person_01")


def make_system():
    world = WorldModel()
    world.update_robot_pose(Pose2D(0.0, 0.0))
    world.set_mission("mission_01", "사람에게 이동")
    world.people["person_01"] = PersonEntity("person_01", Pose2D(2.0, 1.0))
    node = FakeNode()
    navigation = TopicBridgeNavigationAdapter(node)
    results = MockResultQueue()
    orchestrator = VLAOrchestrator(
        world,
        StubLLM(),
        TargetResolver(),
        ActionValidator(),
        ActionDispatcher(
            navigation,
            MockSprayAdapter(results),
            MockReportAdapter(results),
            MockWaitAdapter(results),
        ),
    )
    return world, node, navigation, orchestrator


def publish_result(node, action_id, status):
    message = SimpleNamespace(data=json.dumps({
        "action_id": action_id,
        "target_id": "person_01",
        "status": status,
        "message": f"{status} for test",
    }))
    node.subscriptions["/vla/navigation_result"](message)


@pytest.mark.parametrize(
    ("result_status", "lifecycle_status"),
    [
        ("SUCCEEDED", ActionLifecycleStatus.SUCCEEDED),
        ("ABORTED", ActionLifecycleStatus.ABORTED),
        ("FAILED", ActionLifecycleStatus.FAILED),
        ("CANCELED", ActionLifecycleStatus.CANCELED),
    ],
)
def test_terminal_result_updates_world_and_allows_next_decision(
    result_status, lifecycle_status
):
    world, node, navigation, orchestrator = make_system()
    action = orchestrator.decide_once().validation.action
    publish_result(node, action.action_id, result_status)

    assert orchestrator.process_results(navigation) == 1
    assert world.current_action is None
    assert world.last_action is action
    assert world.last_action.status == lifecycle_status
    assert world.robot.navigation_status == result_status

    orchestrator.decide_once()
    assert orchestrator.llm.calls == 2


def test_unrelated_result_does_not_mutate_world_or_active_goal():
    world, node, navigation, orchestrator = make_system()
    current = orchestrator.decide_once().validation.action
    publish_result(node, "action_stale", "SUCCEEDED")

    assert orchestrator.process_results(navigation) == 0
    assert world.current_action is current
    assert world.last_action is None
    assert world.robot.navigation_status == "IDLE"
    assert any(
        event.event_type == "UNRELATED_RESULT_IGNORED"
        and event.action_id == "action_stale"
        for event in world.event_log
    )

    assert navigation.cancel_current() is True
    cancel = json.loads(node.publishers["/vla/navigation_cancel"].messages[-1].data)
    assert cancel == {"action_id": current.action_id}


def test_duplicate_terminal_result_is_applied_once():
    world, node, navigation, orchestrator = make_system()
    action = orchestrator.decide_once().validation.action

    publish_result(node, action.action_id, "SUCCEEDED")
    assert orchestrator.process_results(navigation) == 1
    publish_result(node, action.action_id, "SUCCEEDED")
    assert orchestrator.process_results(navigation) == 0

    assert sum(
        event.event_type == "NAVIGATION_SUCCEEDED"
        and event.action_id == action.action_id
        for event in world.event_log
    ) == 1
    assert sum(
        event.event_type == "DUPLICATE_RESULT_IGNORED"
        and event.action_id == action.action_id
        for event in world.event_log
    ) == 1
    assert world.current_action is None
    assert world.last_action is action


def test_same_mission_succeeded_navigation_is_not_dispatched_again():
    world, node, navigation, orchestrator = make_system()
    action = orchestrator.decide_once().validation.action
    publish_result(node, action.action_id, "SUCCEEDED")
    assert orchestrator.process_results(navigation) == 1

    duplicate = orchestrator.decide_once()

    assert duplicate.submission is None
    assert duplicate.blocked_reason.startswith("DUPLICATE_ACTION_BLOCKED:")
    assert len(node.publishers["/vla/navigation_goal"].messages) == 1


def test_same_navigation_is_not_dispatched_while_running():
    _, node, _, orchestrator = make_system()
    first = orchestrator.decide_once()

    waiting = orchestrator.decide_once()

    assert first.submission.status.value == "ACCEPTED"
    assert waiting.decision.action == ActionType.WAIT
    assert len(node.publishers["/vla/navigation_goal"].messages) == 1


def test_different_action_for_same_target_is_allowed_after_navigation():
    world, node, navigation, orchestrator = make_system()
    action = orchestrator.decide_once().validation.action
    publish_result(node, action.action_id, "SUCCEEDED")
    assert orchestrator.process_results(navigation) == 1
    orchestrator.llm.decide = lambda mission, snapshot: ActionDecision(
        ActionType.REPORT_PERSON, "도착한 사람을 보고", "person_01"
    )

    report = orchestrator.decide_once()

    assert report.submission.status.value == "ACCEPTED"
    assert report.validation.action.action == ActionType.REPORT_PERSON
    assert len(node.publishers["/vla/navigation_goal"].messages) == 1


def test_same_navigation_is_allowed_for_new_mission():
    world, node, navigation, orchestrator = make_system()
    action = orchestrator.decide_once().validation.action
    publish_result(node, action.action_id, "SUCCEEDED")
    assert orchestrator.process_results(navigation) == 1
    world.set_mission("mission_02", "같은 사람에게 다시 이동")
    world.people["person_01"] = PersonEntity(
        "person_01", Pose2D(2.0, 1.0)
    )

    second = orchestrator.decide_once()

    assert second.submission.status.value == "ACCEPTED"
    assert len(node.publishers["/vla/navigation_goal"].messages) == 2


def test_failed_navigation_keeps_existing_retry_semantics():
    _, node, navigation, orchestrator = make_system()
    action = orchestrator.decide_once().validation.action
    publish_result(node, action.action_id, "FAILED")
    assert orchestrator.process_results(navigation) == 1

    retry = orchestrator.decide_once()

    assert retry.submission.status.value == "ACCEPTED"
    assert len(node.publishers["/vla/navigation_goal"].messages) == 2


def test_aborted_navigation_is_not_redispatched_for_same_mission_target():
    world, node, navigation, orchestrator = make_system()
    action = orchestrator.decide_once().validation.action
    publish_result(node, action.action_id, "ABORTED")

    assert orchestrator.process_results(navigation) == 1
    retry = orchestrator.decide_once()

    assert world.current_action is None
    assert world.last_action is action
    assert world.last_action.status == ActionLifecycleStatus.ABORTED
    assert retry.submission is None
    assert retry.blocked_reason.startswith("DUPLICATE_ACTION_BLOCKED:")
    assert len(node.publishers["/vla/navigation_goal"].messages) == 1


class FakeNavigator:
    def __init__(self):
        self.cancel_calls = 0

    def cancel_navigation(self):
        self.cancel_calls += 1
        return True


@pytest.mark.parametrize(
    ("nav_status", "expected"),
    [
        (GoalStatus.STATUS_SUCCEEDED, "SUCCEEDED"),
        (GoalStatus.STATUS_ABORTED, "ABORTED"),
        (GoalStatus.STATUS_CANCELED, "CANCELED"),
        (GoalStatus.STATUS_UNKNOWN, "FAILED"),
    ],
)
def test_nav2_terminal_status_is_normalized(nav_status, expected):
    published = []
    bridge = SimpleNamespace(
        pending=PendingGoal("action_0001", "person_01"),
        _publish_terminal=lambda *args: published.append(args),
    )

    VLANavigationBridgeNode._navigation_done(bridge, nav_status)

    assert bridge.pending is None
    assert published == [(
        "action_0001",
        "person_01",
        expected,
        f"Nav2 GoalStatus={nav_status}",
    )]


def test_cancel_only_targets_correlated_pending_action():
    navigator = FakeNavigator()
    bridge = SimpleNamespace(
        pending=PendingGoal("action_0002", "person_01"),
        navigator=navigator,
    )

    VLANavigationBridgeNode._cancel_callback(
        bridge, SimpleNamespace(data=json.dumps({"action_id": "action_0001"}))
    )
    assert navigator.cancel_calls == 0

    VLANavigationBridgeNode._cancel_callback(
        bridge, SimpleNamespace(data=json.dumps({"action_id": "action_0002"}))
    )
    assert navigator.cancel_calls == 1
