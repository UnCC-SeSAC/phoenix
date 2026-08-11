from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Iterable, Sequence

from ..domain import (
    Action,
    ActionResult,
    ActionResultStatus,
    ActionSubmission,
    ActionSubmissionStatus,
    ExecutionSource,
    ObservationBatch,
)
from ..ports import ActionResultSource, NavigationPort, PerceptionPort, ReportPort, SprayPort, WaitPort


@dataclass
class MockResultQueue(ActionResultSource):
    _results: deque[ActionResult] = field(default_factory=deque)

    def emit(self, result: ActionResult) -> None:
        self._results.append(result)

    def drain_results(self) -> Sequence[ActionResult]:
        results = tuple(self._results)
        self._results.clear()
        return results


@dataclass
class MockPerceptionAdapter(PerceptionPort):
    batches: deque[ObservationBatch] = field(default_factory=deque)

    @classmethod
    def from_batches(cls, batches: Iterable[ObservationBatch]) -> "MockPerceptionAdapter":
        return cls(deque(batches))

    def read_observations(self) -> ObservationBatch | None:
        return self.batches.popleft() if self.batches else None


@dataclass
class _MockSubmitter:
    result_queue: MockResultQueue
    source: ExecutionSource
    next_result: ActionResultStatus = ActionResultStatus.SUCCEEDED
    processed_ids: set[str] = field(default_factory=set)
    calls: list[Action] = field(default_factory=list)

    def submit(self, action: Action) -> ActionSubmission:
        if action.action_id in self.processed_ids:
            return ActionSubmission(action.action_id, ActionSubmissionStatus.DUPLICATE, "Mock adapter duplicate")
        self.processed_ids.add(action.action_id)
        self.calls.append(action)
        self.result_queue.emit(ActionResult(
            action_id=action.action_id,
            source=self.source,
            status=self.next_result,
            target_id=action.target,
            message=f"Mock {self.source.value}: {self.next_result.value}",
        ))
        return ActionSubmission(action.action_id, ActionSubmissionStatus.ACCEPTED, "Mock accepted")


@dataclass
class MockNavigationAdapter(_MockSubmitter, NavigationPort):
    source: ExecutionSource = ExecutionSource.NAVIGATION

    def cancel_current(self) -> bool:
        return True


@dataclass
class MockSprayAdapter(_MockSubmitter, SprayPort):
    source: ExecutionSource = ExecutionSource.SPRAY

    def stop(self) -> bool:
        return True


@dataclass
class MockReportAdapter(_MockSubmitter, ReportPort):
    source: ExecutionSource = ExecutionSource.REPORT


@dataclass
class MockWaitAdapter(_MockSubmitter, WaitPort):
    source: ExecutionSource = ExecutionSource.WAIT
