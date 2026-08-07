from __future__ import annotations

from typing import Protocol, Sequence

from .domain import Action, ActionDecision, ActionResult, ActionSubmission, ObservationBatch, Pose2D


class LLMPort(Protocol):
    def decide(self, mission: str, world_model: dict) -> ActionDecision: ...


class ExecutionPort(Protocol):
    def submit(self, action: Action) -> ActionSubmission: ...


class NavigationPort(ExecutionPort, Protocol):
    def cancel_current(self) -> bool: ...


class SprayPort(ExecutionPort, Protocol):
    def stop(self) -> bool: ...


class ReportPort(ExecutionPort, Protocol):
    pass


class WaitPort(ExecutionPort, Protocol):
    pass


class PerceptionPort(Protocol):
    def read_observations(self) -> ObservationBatch | None: ...


class ActionResultSource(Protocol):
    def drain_results(self) -> Sequence[ActionResult]: ...
