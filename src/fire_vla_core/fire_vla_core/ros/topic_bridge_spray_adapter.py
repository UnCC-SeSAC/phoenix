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
    ActionType,
    ExecutionSource,
    FireState,
    utc_now_iso,
)
from ..ports import ActionResultSource, SprayPort
from ..world_model import WorldModel


class TopicBridgeSprayAdapter(SprayPort, ActionResultSource):
    """Publish validated spray commands and receive correlated terminal results."""

    def __init__(
        self,
        node,
        world: WorldModel,
        *,
        command_topic: str = "/vla/spray_command",
        result_topic: str = "/vla/spray_result",
        cancel_topic: str = "/vla/spray_cancel",
        max_spray_attempts: int = 2,
    ) -> None:
        if String is None:
            raise RuntimeError("ROS2 std_msgs 패키지를 찾을 수 없습니다.")
        self._node = node
        self._world = world
        self._max_spray_attempts = max_spray_attempts
        self._command_pub = node.create_publisher(String, command_topic, 10)
        self._cancel_pub = node.create_publisher(String, cancel_topic, 10)
        self._result_sub = node.create_subscription(
            String, result_topic, self._result_callback, 10
        )
        self._results: deque[ActionResult] = deque()
        self._active_action_id: str | None = None
        self._submitted_ids: set[str] = set()
        self._submitted_targets: dict[str, str] = {}

    def submit(self, action: Action) -> ActionSubmission:
        if action.action_id in self._submitted_ids:
            return ActionSubmission(
                action.action_id,
                ActionSubmissionStatus.DUPLICATE,
                "이미 제출된 spray action_id입니다.",
            )
        if self._active_action_id is not None:
            return ActionSubmission(
                action.action_id,
                ActionSubmissionStatus.REJECTED,
                "다른 spray action이 실행 중입니다.",
            )
        if action.action != ActionType.EXTINGUISH:
            return ActionSubmission(
                action.action_id,
                ActionSubmissionStatus.REJECTED,
                "EXTINGUISH Action만 제출할 수 있습니다.",
            )
        fire = self._world.fires.get(action.target or "")
        if fire is None:
            return ActionSubmission(
                action.action_id,
                ActionSubmissionStatus.REJECTED,
                "WorldModel에 fire target이 없습니다.",
            )
        if (
            not self._world.fire_is_decision_eligible(fire.id)
            or fire.state != FireState.ACTIVE
            or not fire.robot_within_spray_range
            or fire.spray_count >= self._max_spray_attempts
        ):
            return ActionSubmission(
                action.action_id,
                ActionSubmissionStatus.REJECTED,
                "현재 fire target은 안전한 분사 조건을 충족하지 않습니다.",
            )

        payload = {
            "action_id": action.action_id,
            "mission_id": (
                self._world.mission.id if self._world.mission else None
            ),
            "fire_id": fire.id,
            "command": "SPRAY",
            "timestamp": utc_now_iso(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._command_pub.publish(msg)
        self._active_action_id = action.action_id
        self._submitted_ids.add(action.action_id)
        self._submitted_targets[action.action_id] = fire.id
        return ActionSubmission(
            action.action_id,
            ActionSubmissionStatus.ACCEPTED,
            "spray command topic에 제출했습니다.",
        )

    def stop(self) -> bool:
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
            fire_id = data.get("fire_id")
            message = str(data.get("message", ""))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self._node.get_logger().warning(
                f"spray result parsing failed: {exc}"
            )
            return

        expected_fire_id = self._submitted_targets.get(action_id)
        if expected_fire_id is not None and str(fire_id) != expected_fire_id:
            self._node.get_logger().warning(
                f"spray result target mismatch: action_id={action_id}"
            )
            return

        self._results.append(
            ActionResult(
                action_id=action_id,
                source=ExecutionSource.SPRAY,
                status=status,
                target_id=(
                    expected_fire_id
                    if expected_fire_id is not None
                    else (str(fire_id) if fire_id is not None else None)
                ),
                message=message,
            )
        )
        if action_id == self._active_action_id:
            self._active_action_id = None
