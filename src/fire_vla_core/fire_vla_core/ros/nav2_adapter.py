from __future__ import annotations

from typing import Callable

from ..domain import Action, ActionResult, ActionResultStatus, ActionSubmission, ActionSubmissionStatus, ExecutionSource
from ..ports import NavigationPort

try:
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
except ImportError:
    GoalStatus = PoseStamped = NavigateToPose = ActionClient = None


class ROS2Nav2Adapter(NavigationPort):
    """Non-blocking Nav2 Action adapter. Results are delivered through result_sink."""

    def __init__(self, node, result_sink: Callable[[ActionResult], None], action_name: str = "/navigate_to_pose") -> None:
        if ActionClient is None:
            raise RuntimeError("ROS2/Nav2 Python 패키지를 찾을 수 없습니다.")
        self.node = node
        self.result_sink = result_sink
        self.client = ActionClient(node, NavigateToPose, action_name)
        self._goal_handle = None
        self._active_action: Action | None = None

    def submit(self, action: Action) -> ActionSubmission:
        if action.target_pose is None:
            return ActionSubmission(action.action_id, ActionSubmissionStatus.REJECTED, "target_pose가 없습니다.")
        if self._active_action is not None:
            return ActionSubmission(action.action_id, ActionSubmissionStatus.REJECTED, "Nav2 goal이 이미 실행 중입니다.")
        if not self.client.server_is_ready():
            return ActionSubmission(action.action_id, ActionSubmissionStatus.REJECTED, "Nav2 Action Server가 준비되지 않았습니다.")

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = action.target_pose.x
        goal.pose.pose.position.y = action.target_pose.y
        import math
        goal.pose.pose.orientation.z = math.sin(action.target_pose.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(action.target_pose.yaw / 2.0)
        self._active_action = action
        self.client.send_goal_async(goal).add_done_callback(self._goal_response_cb)
        return ActionSubmission(action.action_id, ActionSubmissionStatus.ACCEPTED, "Nav2 goal 제출")

    def cancel_current(self) -> bool:
        if self._goal_handle is None:
            return False
        self._goal_handle.cancel_goal_async()
        return True

    def _goal_response_cb(self, future) -> None:
        handle = future.result()
        action = self._active_action
        if action is None:
            return
        if not handle.accepted:
            self.result_sink(ActionResult(action.action_id, ExecutionSource.NAVIGATION, ActionResultStatus.ABORTED, action.target, "Nav2 goal 거부"))
            self._active_action = None
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future) -> None:
        wrapped = future.result()
        action = self._active_action
        if action is None:
            return
        mapping = {
            GoalStatus.STATUS_SUCCEEDED: ActionResultStatus.SUCCEEDED,
            GoalStatus.STATUS_ABORTED: ActionResultStatus.ABORTED,
            GoalStatus.STATUS_CANCELED: ActionResultStatus.CANCELED,
        }
        status = mapping.get(wrapped.status, ActionResultStatus.FAILED)
        self.result_sink(ActionResult(action.action_id, ExecutionSource.NAVIGATION, status, action.target, f"Nav2 status={wrapped.status}"))
        self._active_action = None
        self._goal_handle = None
