from __future__ import annotations

import json
from collections import deque
from typing import Sequence

try:
    from std_msgs.msg import String
except ImportError:
    String = None

from ..domain import (
    Action,
    ActionResult,
    ActionResultStatus,
    ActionSubmission,
    ActionSubmissionStatus,
    ExecutionSource,
)
from ..ports import ActionResultSource, NavigationPort


class TopicBridgeNavigationAdapter(NavigationPort, ActionResultSource):
    """Jazzy-side navigation adapter using distro-neutral JSON topics.

    It publishes navigation goals as ``std_msgs/String`` and receives normalized
    terminal results from the Humble-side bridge node. The VLA Core never sees
    ROS messages or Nav2-specific status codes.
    """

    def __init__(
        self,
        node,
        *,
        goal_topic: str = "/vla/navigation_goal",
        result_topic: str = "/vla/navigation_result",
        cancel_topic: str = "/vla/navigation_cancel",
    ) -> None:
        if String is None:
            raise RuntimeError("ROS2 std_msgs 패키지를 찾을 수 없습니다.")
        self._node = node
        self._goal_pub = node.create_publisher(String, goal_topic, 10)
        self._cancel_pub = node.create_publisher(String, cancel_topic, 10)
        self._result_sub = node.create_subscription(
            String,
            result_topic,
            self._result_callback,
            10,
        )
        self._results: deque[ActionResult] = deque()
        self._active_action_id: str | None = None
        self._submitted_ids: set[str] = set()

    def submit(self, action: Action) -> ActionSubmission:
        if action.action_id in self._submitted_ids:
            return ActionSubmission(
                action.action_id,
                ActionSubmissionStatus.DUPLICATE,
                "이미 제출된 navigation action_id입니다.",
            )
        if self._active_action_id is not None:
            return ActionSubmission(
                action.action_id,
                ActionSubmissionStatus.REJECTED,
                "다른 Nav2 goal이 실행 중입니다.",
            )
        if action.target_pose is None:
            return ActionSubmission(
                action.action_id,
                ActionSubmissionStatus.REJECTED,
                "Navigation action에는 target_pose가 필요합니다.",
            )

        payload = {
            "action_id": action.action_id,
            "action": action.action.value,
            "target_id": action.target,
            "target_pose": {
                "x": action.target_pose.x,
                "y": action.target_pose.y,
                "yaw": action.target_pose.yaw,
            },
            "frame_id": "map",
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._goal_pub.publish(msg)
        self._active_action_id = action.action_id
        self._submitted_ids.add(action.action_id)
        return ActionSubmission(
            action.action_id,
            ActionSubmissionStatus.ACCEPTED,
            "Humble Nav2 bridge로 goal을 제출했습니다.",
        )

    def cancel_current(self) -> bool:
        if self._active_action_id is None:
            return False
        msg = String()
        msg.data = json.dumps({"action_id": self._active_action_id})
        self._cancel_pub.publish(msg)
        return True

    def drain_results(self) -> Sequence[ActionResult]:
        results = tuple(self._results)
        self._results.clear()
        return results

    def _result_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            action_id = str(data["action_id"])
            status = ActionResultStatus(str(data["status"]))
            target_id = data.get("target_id")
            message = str(data.get("message", ""))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self._node.get_logger().warning(
                f"navigation result parsing failed: {exc}"
            )
            return

        self._results.append(
            ActionResult(
                action_id=action_id,
                source=ExecutionSource.NAVIGATION,
                status=status,
                target_id=str(target_id) if target_id is not None else None,
                message=message,
            )
        )
        if action_id == self._active_action_id:
            self._active_action_id = None
