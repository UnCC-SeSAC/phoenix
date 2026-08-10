from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .domain import utc_now_iso
from .orchestrator import DecisionCycle


@dataclass(slots=True)
class VLAStatusTracker:
    """Keeps the latest meaningful DecisionCycle metadata for status output."""

    decision: dict[str, Any] | None = None
    validation: dict[str, Any] | None = None
    submission: dict[str, Any] | None = None
    blocked_reason: str = ""

    def update(self, cycle: DecisionCycle) -> None:
        meaningful = any(
            (
                cycle.decision is not None,
                cycle.validation is not None,
                cycle.submission is not None,
                bool(cycle.blocked_reason),
            )
        )
        if not meaningful:
            return

        self.decision = (
            {
                "action": cycle.decision.action.value,
                "target": cycle.decision.target,
                "reason": cycle.decision.reason,
            }
            if cycle.decision is not None
            else None
        )
        self.validation = (
            {
                "approved": cycle.validation.approved,
                "reason": cycle.validation.reason,
            }
            if cycle.validation is not None
            else None
        )
        self.submission = (
            {
                "status": cycle.submission.status.value,
                "detail": cycle.submission.detail or "",
            }
            if cycle.submission is not None
            else None
        )
        self.blocked_reason = cycle.blocked_reason

    def create_payload(self, world_snapshot: dict[str, Any]) -> dict[str, Any]:
        return {
            "timestamp": utc_now_iso(),
            "world_model": world_snapshot,
            "decision": self.decision,
            "validation": self.validation,
            "submission": self.submission,
            "blocked_reason": self.blocked_reason,
        }
