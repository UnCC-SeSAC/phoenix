import json
import sys
import time
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

nav2_msgs = ModuleType("nav2_msgs")
nav2_msgs_action = ModuleType("nav2_msgs.action")


class NavigateToPose:
    class Goal:
        def __init__(self):
            self.pose = None


nav2_msgs_action.NavigateToPose = NavigateToPose
nav2_msgs_action.ComputePathToPose = type("ComputePathToPose", (), {})
nav2_msgs.action = nav2_msgs_action
sys.modules.setdefault("nav2_msgs", nav2_msgs)
sys.modules.setdefault("nav2_msgs.action", nav2_msgs_action)

frontier_package = ModuleType("frontier_exploration_ros2")
frontier_srv = ModuleType("frontier_exploration_ros2.srv")


class ControlExploration:
    class Request:
        STATE_IDLE = 0
        STATE_RUNNING = 1
        ACTION_START = 1
        ACTION_STOP = 2


frontier_srv.ControlExploration = ControlExploration
frontier_package.srv = frontier_srv
sys.modules.setdefault("frontier_exploration_ros2", frontier_package)
sys.modules.setdefault("frontier_exploration_ros2.srv", frontier_srv)

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from uncc_example.mission_executor import MissionExecutor
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time as TimeMsg
from uncc_example.nav2_navigator import NavigationOutcome
from uncc_example.vla_navigation_bridge_node import PendingGoal, VLANavigationBridgeNode


class FakeFuture:
    def add_done_callback(self, callback):
        self.callback = callback


class FakeNavClient:
    def __init__(self):
        self.goals = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return FakeFuture()


class FakeSuppressionClient(FakeNavClient):
    def send_goal_async(self, goal, feedback_callback=None):
        self.goals.append(goal)
        return FakeFuture()


def pose():
    result = PoseStamped()
    result.header.frame_id = "map"
    result.pose.position.x = 1.0
    return result


def mission_executor(mode):
    client = FakeNavClient()
    executor = SimpleNamespace(
        control_mode=mode,
        state="PERSON_DETECTED",
        _fire_cycle_active=False,
        _nav_goal_xy=None,
        _nav_goal_pending=False,
        _nav_cancel_pending=False,
        _nav_goal_handle=None,
        _nav_client=client,
        get_logger=lambda: SimpleNamespace(warn=lambda *args, **kwargs: None),
    )
    executor._cancel_nav_goal = lambda: MissionExecutor._cancel_nav_goal(executor)
    executor._nav_goal_response = lambda future: None
    return executor, client


def navigation_bridge(mode):
    goals = []
    bridge = SimpleNamespace(
        _control_mode=mode,
        map_frame="map",
        pending=None,
        completed_results={},
        navigator=SimpleNamespace(navigate=lambda goal, *callbacks: goals.append(goal)),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=TimeMsg)
        ),
        get_logger=lambda: SimpleNamespace(
            warning=lambda *args, **kwargs: None,
            info=lambda *args, **kwargs: None,
        ),
    )
    bridge._publish_terminal = lambda *args: None
    bridge._navigation_done = lambda status: None
    bridge._navigation_goal_response = lambda accepted, goal_uuid: None
    bridge._navigation_feedback = lambda current_pose, distance_remaining: None
    return bridge, goals


def navigation_message():
    msg = String()
    msg.data = json.dumps({
        "action_id": "action_1",
        "target_id": "fire_1",
        "frame_id": "map",
        "target_pose": {"x": 1.0, "y": 0.0, "yaw": 0.0},
    })
    return msg


def test_none_mode_blocks_vla_and_fsm_navigation():
    bridge, vla_goals = navigation_bridge("NONE")
    VLANavigationBridgeNode._goal_callback(bridge, navigation_message())
    executor, fsm_client = mission_executor("NONE")
    MissionExecutor._send_nav_goal(executor, pose())
    assert vla_goals == []
    assert fsm_client.goals == []


def test_vla_mode_allows_only_vla_navigation():
    bridge, vla_goals = navigation_bridge("VLA")
    VLANavigationBridgeNode._goal_callback(bridge, navigation_message())
    executor, fsm_client = mission_executor("VLA")
    MissionExecutor._send_nav_goal(executor, pose())
    assert len(vla_goals) == 1
    assert fsm_client.goals == []


def test_rule_based_mode_allows_fsm_and_frontier_gate_only():
    bridge, vla_goals = navigation_bridge("RULE_BASED")
    VLANavigationBridgeNode._goal_callback(bridge, navigation_message())
    executor, fsm_client = mission_executor("RULE_BASED")
    MissionExecutor._send_nav_goal(executor, pose())
    source = (
        Path(__file__).parents[2]
        / "frontier_exploration_ros2/src/frontier_explorer_node.cpp"
    ).read_text(encoding="utf-8")
    assert vla_goals == []
    assert len(fsm_client.goals) == 1
    assert 'control_mode_ != "RULE_BASED"' in source


def test_fsm_frontier_handoff_still_waits_for_idle_before_goal():
    source = (
        Path(__file__).parents[1] / "uncc_example/mission_executor.py"
    ).read_text(encoding="utf-8")
    stop_check = "self._frontier_state != ControlExploration.Request.STATE_IDLE"
    assert stop_check in source
    assert source.index(stop_check) < source.index("self._send_nav_goal(self.current_target)")


def test_rule_based_mode_is_required_for_fsm_suppression():
    client = FakeSuppressionClient()
    executor = SimpleNamespace(
        control_mode="NONE",
        _fire_cycle_active=False,
        _fire_action_client=client,
        _fire_goal_pending=False,
        _fire_cancel_pending=False,
        _suppress_feedback_callback=lambda feedback: None,
        _suppress_goal_response=lambda future: None,
        get_logger=lambda: SimpleNamespace(warn=lambda *args, **kwargs: None),
    )
    MissionExecutor._call_fire_suppression(executor)
    assert client.goals == []
    executor.control_mode = "RULE_BASED"
    MissionExecutor._call_fire_suppression(executor)
    assert len(client.goals) == 1


@pytest.mark.parametrize(
    ("nav_status", "terminal"),
    [
        (GoalStatus.STATUS_SUCCEEDED, "SUCCEEDED"),
        (GoalStatus.STATUS_ABORTED, "ABORTED"),
        (GoalStatus.STATUS_CANCELED, "CANCELED"),
    ],
)
def test_navigation_terminal_log_preserves_result_details(nav_status, terminal):
    logs = []
    published = []
    bridge = SimpleNamespace(
        pending=PendingGoal("action_7", "fire_2", 1.25, -0.5, 0.75, time.monotonic() - 1.0),
        _last_feedback_pose=(0.5, -0.25),
        _last_distance_remaining=0.8,
        get_logger=lambda: SimpleNamespace(info=logs.append),
        _publish_terminal=lambda *args: published.append(args),
    )

    VLANavigationBridgeNode._navigation_done(
        bridge,
        NavigationOutcome(
            status=nav_status,
            goal_uuid="001122",
            error_code=42,
            error_msg="nav detail",
        ),
    )

    assert published[0][2] == terminal
    assert f"status={terminal}" in logs[0]
    assert "action_id=action_7" in logs[0]
    assert "goal_uuid=001122" in logs[0]
    assert "target_pose=(1.250, -0.500, 0.750)" in logs[0]
    assert "error_code=42" in logs[0]
    assert "error_msg=nav detail" in logs[0]
    assert "last_pose=(0.500, -0.250)" in logs[0]
    assert "distance_remaining=0.800" in logs[0]
    assert "elapsed_sec=" in logs[0]
