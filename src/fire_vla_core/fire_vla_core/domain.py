from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from enum import Enum
from math import atan2, hypot
from typing import Any


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def utc_now_iso() -> str:
    return utc_now().isoformat()


class MissionStatus(str, Enum):
    READY = "READY"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    ABORTED = "ABORTED"


class MissionScope(str, Enum):
    FIRE_ONLY = "FIRE_ONLY"
    PERSON_FIRE = "PERSON_FIRE"
    FULL_EXPLORATION = "FULL_EXPLORATION"


class ExplorationStatus(str, Enum):
    NOT_STARTED = "NOT_STARTED"
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"


class PersonState(str, Enum):
    DETECTED = "DETECTED"
    REPORTED = "REPORTED"
    LOST = "LOST"  # POST-MVP: no automatic transition yet.


class FireState(str, Enum):
    ACTIVE = "ACTIVE"
    PENDING_VERIFICATION = "PENDING_VERIFICATION"
    EXTINGUISHED = "EXTINGUISHED"
    INACCESSIBLE = "INACCESSIBLE"


class ActionType(str, Enum):
    NAVIGATE_TO = "NAVIGATE_TO"
    REPORT_PERSON = "REPORT_PERSON"
    EXTINGUISH = "EXTINGUISH"
    SEARCH = "SEARCH"
    WAIT = "WAIT"
    RETURN_HOME = "RETURN_HOME"


class ActionLifecycleStatus(str, Enum):
    PROPOSED = "PROPOSED"
    VALIDATED = "VALIDATED"
    EXECUTING = "EXECUTING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    ABORTED = "ABORTED"
    CANCELED = "CANCELED"
    TIMED_OUT = "TIMED_OUT"
    REJECTED = "REJECTED"


class ActionSubmissionStatus(str, Enum):
    ACCEPTED = "ACCEPTED"
    REJECTED = "REJECTED"
    DUPLICATE = "DUPLICATE"


class ActionResultStatus(str, Enum):
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    ABORTED = "ABORTED"
    CANCELED = "CANCELED"
    TIMED_OUT = "TIMED_OUT"


class ExecutionSource(str, Enum):
    NAVIGATION = "NAVIGATION"
    SPRAY = "SPRAY"
    REPORT = "REPORT"
    WAIT = "WAIT"
    DISPATCHER = "DISPATCHER"


@dataclass(frozen=True, slots=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0

    def distance_to(self, other: "Pose2D") -> float:
        return hypot(self.x - other.x, self.y - other.y)

    def yaw_to(self, other: "Pose2D") -> float:
        return atan2(other.y - self.y, other.x - self.x)


@dataclass(frozen=True, slots=True)
class SemanticObservation:
    entity_id: str
    class_name: str
    confidence: float
    position: Pose2D
    observed_at: str
    size: str | None = None
    blocks_route_to: str | None = None


@dataclass(frozen=True, slots=True)
class ObservationBatch:
    observed_at: str
    observations: tuple[SemanticObservation, ...]
    frame_valid: bool = True
    detector_healthy: bool = True


@dataclass(slots=True)
class Mission:
    id: str
    text: str
    status: MissionStatus = MissionStatus.READY
    scope: MissionScope | None = None
    target_fire_id: str | None = None
    target_person_id: str | None = None


@dataclass(slots=True)
class PersonEntity:
    id: str
    position: Pose2D
    confidence: float = 1.0
    state: PersonState = PersonState.DETECTED
    reported: bool = False
    first_seen: str = field(default_factory=utc_now_iso)
    last_seen: str = field(default_factory=utc_now_iso)


@dataclass(slots=True)
class FireEntity:
    id: str
    position: Pose2D
    confidence: float = 1.0
    size: str = "UNKNOWN"
    state: FireState = FireState.ACTIVE
    blocks_route_to: str | None = None
    threatens_person: bool = False
    threatened_person_id: str | None = None
    spray_count: int = 0
    robot_within_spray_range: bool = False
    first_seen: str = field(default_factory=utc_now_iso)
    last_seen: str = field(default_factory=utc_now_iso)
    verification_started_at: str | None = None
    verification_valid_observations: int = 0


@dataclass(slots=True)
class RobotState:
    pose: Pose2D | None = None
    pose_updated_at: str | None = None
    navigation_status: str = "IDLE"
    home_pose: Pose2D | None = None


@dataclass(frozen=True, slots=True)
class ActionDecision:
    action: ActionType
    reason: str
    target: str | None = None
    mission_scope: MissionScope = MissionScope.FULL_EXPLORATION

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "ActionDecision":
        try:
            action_type = ActionType(data["action"])
        except (KeyError, ValueError) as exc:
            raise ValueError("유효한 action 필드가 필요합니다.") from exc
        try:
            mission_scope = MissionScope(data["mission_scope"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("유효한 mission_scope 필드가 필요합니다.") from exc
        return cls(
            action=action_type,
            target=data.get("target"),
            reason=str(data.get("reason", "")).strip(),
            mission_scope=mission_scope,
        )


@dataclass(slots=True)
class Action:
    action_id: str
    action: ActionType
    reason: str
    target: str | None = None
    target_pose: Pose2D | None = None
    status: ActionLifecycleStatus = ActionLifecycleStatus.PROPOSED

    @property
    def is_physical(self) -> bool:
        return self.action in {
            ActionType.NAVIGATE_TO,
            ActionType.SEARCH,
            ActionType.RETURN_HOME,
            ActionType.EXTINGUISH,
        }

    def to_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["action"] = self.action.value
        data["status"] = self.status.value
        return data


@dataclass(frozen=True, slots=True)
class ActionSubmission:
    action_id: str
    status: ActionSubmissionStatus
    detail: str | None = None


@dataclass(frozen=True, slots=True)
class ActionResult:
    action_id: str
    source: ExecutionSource
    status: ActionResultStatus
    target_id: str | None = None
    message: str = ""
    timestamp: str = field(default_factory=utc_now_iso)


@dataclass(slots=True)
class Event:
    event_type: str
    timestamp: str = field(default_factory=utc_now_iso)
    entity_id: str | None = None
    action_id: str | None = None
    detail: str = ""
    data: dict[str, Any] = field(default_factory=dict)
