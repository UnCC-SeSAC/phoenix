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
    PersonState,
    utc_now_iso,
)
from ..ports import ActionResultSource, ReportPort
from ..world_model import WorldModel


class TopicBridgePersonReportAdapter(ReportPort, ActionResultSource):
    """Publish authoritative person reports and receive terminal results."""

    def __init__(
        self,
        node,
        world: WorldModel,
        *,
        report_topic: str = "/vla/person_report",
        result_topic: str = "/vla/person_report_result",
    ) -> None:
        if String is None:
            raise RuntimeError("ROS2 std_msgs 패키지를 찾을 수 없습니다.")
        self._node = node
        self._world = world
        self._report_pub = node.create_publisher(String, report_topic, 10)
        self._result_sub = node.create_subscription(
            String, result_topic, self._result_callback, 10
        )
        self._results: deque[ActionResult] = deque()
        self._submitted_ids: set[str] = set()
        self._submitted_targets: dict[str, str] = {}
        self._auto_reported: set[tuple[str, str]] = set()

    def publish_new_people(self) -> int:
        mission = self._world.mission
        if mission is None:
            return 0
        published = 0
        for person in self._world.people.values():
            key = (mission.id, person.id)
            if person.reported or key in self._auto_reported:
                continue
            action_id = f"auto_report:{mission.id}:{person.id}"
            self._publish_payload(action_id, person)
            self._auto_reported.add(key)
            person.reported = True
            person.state = PersonState.REPORTED
            published += 1
        return published

    def submit(self, action: Action) -> ActionSubmission:
        if action.action_id in self._submitted_ids:
            return ActionSubmission(
                action.action_id,
                ActionSubmissionStatus.DUPLICATE,
                "이미 제출된 person report action_id입니다.",
            )
        if action.action != ActionType.REPORT_PERSON:
            return ActionSubmission(
                action.action_id,
                ActionSubmissionStatus.REJECTED,
                "REPORT_PERSON Action만 제출할 수 있습니다.",
            )
        person = self._world.people.get(action.target or "")
        if person is None or person.reported:
            return ActionSubmission(
                action.action_id,
                ActionSubmissionStatus.REJECTED,
                "보고 가능한 person target이 없습니다.",
            )

        self._publish_payload(action.action_id, person)
        self._submitted_ids.add(action.action_id)
        self._submitted_targets[action.action_id] = person.id
        return ActionSubmission(
            action.action_id,
            ActionSubmissionStatus.ACCEPTED,
            "person report topic에 제출했습니다.",
        )

    def _publish_payload(self, action_id: str, person) -> None:
        payload = {
            "action_id": action_id,
            "mission_id": (
                self._world.mission.id if self._world.mission else None
            ),
            "person_id": person.id,
            "map_position": {
                "x": person.position.x,
                "y": person.position.y,
            },
            "confidence": person.confidence,
            "timestamp": utc_now_iso(),
            "frame_id": "map",
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._report_pub.publish(msg)

    def drain_results(self) -> Sequence[ActionResult]:
        results = tuple(self._results)
        self._results.clear()
        return results

    def _result_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            action_id = str(data["action_id"])
            status = ActionResultStatus(str(data["status"]))
            person_id = data.get("person_id")
            message = str(data.get("message", ""))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self._node.get_logger().warning(
                f"person report result parsing failed: {exc}"
            )
            return

        expected_person_id = self._submitted_targets.get(action_id)
        if (
            expected_person_id is not None
            and str(person_id) != expected_person_id
        ):
            self._node.get_logger().warning(
                "person report result target mismatch: "
                f"action_id={action_id}"
            )
            return

        self._results.append(
            ActionResult(
                action_id=action_id,
                source=ExecutionSource.REPORT,
                status=status,
                target_id=(
                    expected_person_id
                    if expected_person_id is not None
                    else (
                        str(person_id)
                        if person_id is not None
                        else None
                    )
                ),
                message=message,
            )
        )
