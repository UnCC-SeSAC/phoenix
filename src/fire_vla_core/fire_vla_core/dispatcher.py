from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

from .domain import Action, ActionSubmission, ActionSubmissionStatus, ActionType
from .ports import NavigationPort, ReportPort, SprayPort, WaitPort


@dataclass(slots=True)
class ActionDispatcher:
    navigation: NavigationPort
    spray: SprayPort
    report: ReportPort
    waiter: WaitPort
    idempotency_cache_size: int = 256
    _submitted_ids: deque[str] = field(default_factory=deque, init=False, repr=False)
    _submitted_id_set: set[str] = field(default_factory=set, init=False, repr=False)

    def submit(self, action: Action) -> ActionSubmission:
        if action.action_id in self._submitted_id_set:
            return ActionSubmission(action.action_id, ActionSubmissionStatus.DUPLICATE, "이미 제출된 action_id입니다.")

        port = self._select_port(action.action)
        submission = port.submit(action)
        if submission.status == ActionSubmissionStatus.ACCEPTED:
            self._remember(action.action_id)
        return submission

    def cancel_current(self) -> bool:
        nav = self.navigation.cancel_current()
        spray = self.spray.stop()
        return nav or spray

    def _select_port(self, action_type: ActionType):
        if action_type in {ActionType.NAVIGATE_TO, ActionType.SEARCH, ActionType.RETURN_HOME}:
            return self.navigation
        if action_type == ActionType.EXTINGUISH:
            return self.spray
        if action_type == ActionType.REPORT_PERSON:
            return self.report
        if action_type == ActionType.WAIT:
            return self.waiter
        raise ValueError(f"지원하지 않는 행동입니다: {action_type}")

    def _remember(self, action_id: str) -> None:
        self._submitted_ids.append(action_id)
        self._submitted_id_set.add(action_id)
        while len(self._submitted_ids) > self.idempotency_cache_size:
            expired = self._submitted_ids.popleft()
            self._submitted_id_set.discard(expired)
