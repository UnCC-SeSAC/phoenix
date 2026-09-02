from dataclasses import dataclass
from typing import Callable, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup


@dataclass(frozen=True)
class NavigationOutcome:
    status: int
    goal_uuid: str | None = None
    error_code: int = 0
    error_msg: str = ''


class Nav2Navigator:
    """
    Thin asynchronous wrapper around Nav2 actions.
    """

    def __init__(self, node):
        self.node = node
        self.callback_group = ReentrantCallbackGroup()

        self.compute_path_client = ActionClient(
            node,
            ComputePathToPose,
            '/compute_path_to_pose',
            callback_group=self.callback_group,
        )

        self.navigate_client = ActionClient(
            node,
            NavigateToPose,
            '/navigate_to_pose',
            callback_group=self.callback_group,
        )

        self._nav_goal_handle = None

    def planner_ready(self) -> bool:
        return self.compute_path_client.server_is_ready()

    def navigator_ready(self) -> bool:
        return self.navigate_client.server_is_ready()

    def request_path(
        self,
        start: PoseStamped,
        goal: PoseStamped,
        on_result: Callable[[Optional[Path]], None],
    ):
        if not self.planner_ready():
            on_result(None)
            return

        msg = ComputePathToPose.Goal()
        msg.start = start
        msg.goal = goal
        msg.use_start = True
        msg.planner_id = ''

        future = self.compute_path_client.send_goal_async(msg)

        def goal_response_done(goal_future):
            try:
                goal_handle = goal_future.result()
            except Exception as exc:
                self.node.get_logger().warn(
                    f'ComputePathToPose goal error: {exc}'
                )
                on_result(None)
                return

            if not goal_handle.accepted:
                on_result(None)
                return

            result_future = goal_handle.get_result_async()

            def result_done(path_future):
                try:
                    wrapped = path_future.result()
                    if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                        on_result(wrapped.result.path)
                    else:
                        on_result(None)
                except Exception as exc:
                    self.node.get_logger().warn(
                        f'ComputePathToPose result error: {exc}'
                    )
                    on_result(None)

            result_future.add_done_callback(result_done)

        future.add_done_callback(goal_response_done)

    def navigate(
        self,
        goal: PoseStamped,
        on_result: Callable[[NavigationOutcome], None],
        on_goal_response: Callable[[bool, str | None], None] | None = None,
        on_feedback: Callable[[object, float], None] | None = None,
    ):
        if not self.navigator_ready():
            if on_goal_response is not None:
                on_goal_response(False, None)
            on_result(NavigationOutcome(
                GoalStatus.STATUS_ABORTED,
                error_msg='NavigateToPose server is not ready',
            ))
            return

        msg = NavigateToPose.Goal()
        msg.pose = goal

        def feedback_done(feedback_msg):
            if on_feedback is None:
                return
            feedback = feedback_msg.feedback
            on_feedback(
                feedback.current_pose,
                float(feedback.distance_remaining),
            )

        future = self.navigate_client.send_goal_async(
            msg, feedback_callback=feedback_done
        )

        def goal_response_done(goal_future):
            try:
                goal_handle = goal_future.result()
            except Exception as exc:
                self.node.get_logger().warn(
                    f'NavigateToPose goal error: {exc}'
                )
                if on_goal_response is not None:
                    on_goal_response(False, None)
                on_result(NavigationOutcome(
                    GoalStatus.STATUS_ABORTED,
                    error_msg=str(exc),
                ))
                return

            if not goal_handle.accepted:
                if on_goal_response is not None:
                    on_goal_response(False, None)
                on_result(NavigationOutcome(
                    GoalStatus.STATUS_ABORTED,
                    error_msg='NavigateToPose goal rejected',
                ))
                return

            self._nav_goal_handle = goal_handle
            goal_uuid = bytes(goal_handle.goal_id.uuid).hex()
            if on_goal_response is not None:
                on_goal_response(True, goal_uuid)

            result_future = goal_handle.get_result_async()

            def result_done(nav_future):
                try:
                    wrapped = nav_future.result()
                    result = wrapped.result
                    on_result(NavigationOutcome(
                        status=wrapped.status,
                        goal_uuid=goal_uuid,
                        error_code=int(getattr(result, 'error_code', 0)),
                        error_msg=str(getattr(result, 'error_msg', '')),
                    ))
                except Exception as exc:
                    self.node.get_logger().warn(
                        f'NavigateToPose result error: {exc}'
                    )
                    on_result(NavigationOutcome(
                        GoalStatus.STATUS_ABORTED,
                        goal_uuid=goal_uuid,
                        error_msg=str(exc),
                    ))
                finally:
                    self._nav_goal_handle = None

            result_future.add_done_callback(result_done)

        future.add_done_callback(goal_response_done)

    def cancel_navigation(self, on_done=None) -> bool:
        if self._nav_goal_handle is None:
            return False

        future = self._nav_goal_handle.cancel_goal_async()

        if on_done is not None:
            future.add_done_callback(lambda _: on_done())

        return True
