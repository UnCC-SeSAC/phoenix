from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, field
from datetime import datetime
from math import isclose
from typing import Any, Iterable

from .domain import (
    Action,
    ActionLifecycleStatus,
    ActionResult,
    ActionResultStatus,
    ActionSubmission,
    ActionSubmissionStatus,
    Event,
    ExecutionSource,
    ExplorationStatus,
    FireEntity,
    FireState,
    Mission,
    MissionStatus,
    ObservationBatch,
    PersonEntity,
    PersonState,
    Pose2D,
    RobotState,
    SemanticObservation,
    utc_now,
    utc_now_iso,
)


@dataclass(slots=True)
class WorldModelConfig:
    person_confidence_threshold: float = 0.50
    fire_confidence_threshold: float = 0.40
    spray_range_m: float = 0.80
    person_fire_risk_distance_m: float = 0.10
    max_event_log_entries: int = 500
    verification_required_observations: int = 3
    verification_timeout_sec: float = 5.0
    verification_delay_sec: float = 0.5
    verification_min_confidence: float = 0.5
    observation_max_age_sec: float = 1.0
    robot_pose_max_age_sec: float = 0.5
    processed_result_cache_size: int = 256


@dataclass
class WorldModel:
    config: WorldModelConfig = field(default_factory=WorldModelConfig)
    mission: Mission | None = None
    exploration_status: ExplorationStatus = ExplorationStatus.NOT_STARTED
    perception_ready: bool = False
    robot: RobotState = field(default_factory=RobotState)
    people: dict[str, PersonEntity] = field(default_factory=dict)
    fires: dict[str, FireEntity] = field(default_factory=dict)
    current_action: Action | None = None
    last_action: Action | None = None
    pending_actions: dict[str, Action] = field(default_factory=dict)
    event_log: list[Event] = field(default_factory=list)
    unexplored_zones: list[dict[str, Any]] = field(default_factory=list)
    _processed_terminal_action_ids: deque[str] = field(default_factory=deque, repr=False)
    _processed_terminal_action_id_set: set[str] = field(default_factory=set, repr=False)

    def set_mission(self, mission_id: str, text: str) -> None:
        self.people.clear()
        self.fires.clear()
        self.mission = Mission(id=mission_id, text=text, status=MissionStatus.RUNNING)
        self.exploration_status = ExplorationStatus.RUNNING
        if self.robot.pose and self.robot.home_pose is None:
            self.robot.home_pose = self.robot.pose
        self._event("MISSION_STARTED", detail=text)

    def abort_mission(self, detail: str = "운영자 중단") -> None:
        if self.mission:
            self.mission.status = MissionStatus.ABORTED
        self._event("MISSION_ABORTED", detail=detail)

    def mark_exploration_completed(self) -> None:
        self.exploration_status = ExplorationStatus.COMPLETED
        self._event("EXPLORATION_COMPLETED")

    def update_robot_pose(self, pose: Pose2D, updated_at: str | None = None) -> None:
        self.robot.pose = pose
        self.robot.pose_updated_at = updated_at or utc_now_iso()
        if self.mission and self.robot.home_pose is None:
            self.robot.home_pose = pose
        self._refresh_spatial_flags()

    def update_observation_batch(self, batch: ObservationBatch) -> None:
        age = self._timestamp_age_seconds(batch.observed_at)
        if age is None:
            self._event(
                "OBSERVATION_INVALID",
                detail="observed_at timestamp가 유효하지 않습니다.",
            )
            return
        if age < 0:
            self._event("FUTURE_OBSERVATION_IGNORED", detail=batch.observed_at)
            return
        if age > self.config.observation_max_age_sec:
            self._event("STALE_OBSERVATION_IGNORED", detail=batch.observed_at)
            return
        self.perception_ready = True
        if not batch.frame_valid or not batch.detector_healthy:
            self._event("OBSERVATION_INVALID", detail="frame_valid 또는 detector_healthy가 false")
            self._process_verification_timeouts(batch.observed_at)
            return

        seen_people: set[str] = set()
        seen_fires: set[str] = set()
        for observation in batch.observations:
            class_name = observation.class_name.lower()
            if class_name == "person" and observation.confidence >= self.config.person_confidence_threshold:
                seen_people.add(observation.entity_id)
                self._upsert_person(observation)
            elif class_name == "fire" and observation.confidence >= self.config.fire_confidence_threshold:
                seen_fires.add(observation.entity_id)
                self._upsert_fire(observation)

        self._update_fire_verification(seen_fires, batch.observed_at)
        self._refresh_spatial_flags()

    def update_observations(self, observations: Iterable[SemanticObservation], timestamp: str | None = None) -> None:
        """Compatibility helper for non-ROS callers using already normalized DTOs."""
        observed_at = timestamp or utc_now_iso()
        self.update_observation_batch(ObservationBatch(observed_at, tuple(observations)))

    def set_blocks_route(self, fire_id: str, target_id: str | None) -> None:
        if fire_id in self.fires:
            self.fires[fire_id].blocks_route_to = target_id
            self._event("SPATIAL_RELATION_UPDATED", entity_id=fire_id, detail=f"blocks_route_to={target_id}")

    def apply_submission(self, action: Action, submission: ActionSubmission) -> None:
        if submission.status == ActionSubmissionStatus.ACCEPTED:
            action.status = ActionLifecycleStatus.EXECUTING
            self.pending_actions[action.action_id] = action
            if action.is_physical:
                self.current_action = action
            self._event("ACTION_ACCEPTED", action_id=action.action_id, entity_id=action.target, detail=submission.detail or "", data=action.to_dict())
            return

        action.status = ActionLifecycleStatus.REJECTED
        self.last_action = action
        self._event(f"ACTION_{submission.status.value}", action_id=action.action_id, entity_id=action.target, detail=submission.detail or "")

    def apply_action_result(self, result: ActionResult) -> bool:
        if result.action_id in self._processed_terminal_action_id_set:
            self._event("DUPLICATE_RESULT_IGNORED", action_id=result.action_id, detail=result.message)
            return False

        action = self.pending_actions.get(result.action_id)
        if action is None and self.current_action and self.current_action.action_id == result.action_id:
            action = self.current_action
        if action is None:
            self._event(
                "UNRELATED_RESULT_IGNORED",
                action_id=result.action_id,
                entity_id=result.target_id,
                detail=result.message,
            )
            return False

        self._remember_processed_result(result.action_id)
        self.pending_actions.pop(result.action_id, None)
        action.status = self._lifecycle_from_result(result.status)

        if result.source == ExecutionSource.NAVIGATION:
            self.robot.navigation_status = result.status.value

        if result.source == ExecutionSource.REPORT and result.status == ActionResultStatus.SUCCEEDED and result.target_id in self.people:
            person = self.people[result.target_id]
            person.reported = True
            person.state = PersonState.REPORTED

        if result.source == ExecutionSource.SPRAY and result.target_id in self.fires:
            fire = self.fires[result.target_id]
            fire.spray_count += 1
            if result.status == ActionResultStatus.SUCCEEDED:
                fire.state = FireState.PENDING_VERIFICATION
                fire.verification_started_at = result.timestamp
                fire.verification_valid_observations = 0
                self._event("SUPPRESSION_PENDING_VERIFICATION", entity_id=fire.id, action_id=result.action_id)
            else:
                fire.state = FireState.ACTIVE
                self._event("SUPPRESSION_ATTEMPT_FAILED", entity_id=fire.id, action_id=result.action_id, detail=result.message)

        self._event(f"{result.source.value}_{result.status.value}", entity_id=result.target_id, action_id=result.action_id, detail=result.message)

        if self.current_action and self.current_action.action_id == result.action_id:
            self.last_action = self.current_action
            self.current_action = None
        elif action:
            self.last_action = action
        return True

    def mark_fire_inaccessible(self, fire_id: str) -> None:
        fire = self.fires[fire_id]
        fire.state = FireState.INACCESSIBLE
        self._event("FIRE_INACCESSIBLE", entity_id=fire_id)

    def create_snapshot(self) -> dict[str, Any]:
        return {
            "mission": self._serialize(self.mission),
            "exploration_status": self.exploration_status.value,
            "perception_ready": self.perception_ready,
            "robot": self._serialize(self.robot),
            "people": [self._serialize(p) for p in self.people.values()],
            "fires": [self._serialize(f) for f in self.fires.values()],
            "current_action": self._serialize(self.current_action),
            "last_action": self._serialize(self.last_action),
            "pending_action_ids": sorted(self.pending_actions),
            "unexplored_zones": self.unexplored_zones,
            "recent_events": [self._serialize(e) for e in self.event_log[-20:]],
        }

    def complete_mission_if_resolved(self) -> bool:
        if not self.mission_goals_resolved():
            return False
        assert self.mission is not None
        self.mission.status = MissionStatus.COMPLETED
        self._event("MISSION_COMPLETED")
        return True

    def mission_goals_resolved(self) -> bool:
        if not self.mission or self.mission.status != MissionStatus.RUNNING:
            return False
        if not self.perception_ready or self.exploration_status != ExplorationStatus.COMPLETED:
            return False
        if self.current_action is not None:
            return False
        people_done = all(p.state == PersonState.REPORTED for p in self.people.values())
        fires_done = all(f.state in {FireState.EXTINGUISHED, FireState.INACCESSIBLE} for f in self.fires.values())
        return people_done and fires_done

    def find_unexplored_zone(self, zone_id: str | None) -> dict[str, Any] | None:
        if zone_id is not None:
            return next((z for z in self.unexplored_zones if z.get("id") == zone_id), None)
        return self.unexplored_zones[0] if self.unexplored_zones else None

    def _upsert_person(self, observation: SemanticObservation) -> None:
        if observation.entity_id not in self.people:
            self.people[observation.entity_id] = PersonEntity(
                observation.entity_id,
                observation.position,
                confidence=observation.confidence,
                first_seen=observation.observed_at,
                last_seen=observation.observed_at,
            )
            self._event("PERSON_DETECTED", entity_id=observation.entity_id, data={"position": asdict(observation.position)})
            return
        person = self.people[observation.entity_id]
        person.position = observation.position
        person.confidence = observation.confidence
        person.last_seen = observation.observed_at

    def _upsert_fire(self, observation: SemanticObservation) -> None:
        if observation.entity_id not in self.fires:
            self.fires[observation.entity_id] = FireEntity(
                observation.entity_id,
                observation.position,
                confidence=observation.confidence,
                size=(observation.size or "UNKNOWN").upper(),
                blocks_route_to=observation.blocks_route_to,
                first_seen=observation.observed_at,
                last_seen=observation.observed_at,
            )
            self._event("FIRE_DETECTED", entity_id=observation.entity_id, data={"position": asdict(observation.position)})
            return
        fire = self.fires[observation.entity_id]
        fire.position = observation.position
        fire.confidence = observation.confidence
        fire.last_seen = observation.observed_at
        fire.blocks_route_to = observation.blocks_route_to or fire.blocks_route_to
        if fire.state == FireState.PENDING_VERIFICATION and observation.confidence >= self.config.verification_min_confidence:
            fire.state = FireState.ACTIVE
            fire.verification_valid_observations = 0
            fire.verification_started_at = None
            self._event("SUPPRESSION_VERIFICATION_FAILED", entity_id=fire.id, detail="화점이 재탐지되어 ACTIVE로 복귀")
        elif fire.state != FireState.EXTINGUISHED:
            fire.state = FireState.ACTIVE

    def _update_fire_verification(self, seen_fires: set[str], observed_at: str) -> None:
        for fire in self.fires.values():
            if fire.state != FireState.PENDING_VERIFICATION:
                continue
            if fire.id in seen_fires:
                continue
            if not fire.verification_started_at:
                fire.verification_started_at = observed_at
            elapsed = self._elapsed_seconds(fire.verification_started_at, observed_at)
            if elapsed < self.config.verification_delay_sec:
                continue
            if elapsed > self.config.verification_timeout_sec:
                fire.state = FireState.ACTIVE
                fire.verification_valid_observations = 0
                fire.verification_started_at = None
                self._event("SUPPRESSION_VERIFICATION_TIMED_OUT", entity_id=fire.id)
                continue
            fire.verification_valid_observations += 1
            if fire.verification_valid_observations >= self.config.verification_required_observations:
                fire.state = FireState.EXTINGUISHED
                fire.verification_started_at = None
                self._event("FIRE_EXTINGUISHED", entity_id=fire.id)

    def _process_verification_timeouts(self, observed_at: str) -> None:
        for fire in self.fires.values():
            if fire.state != FireState.PENDING_VERIFICATION or not fire.verification_started_at:
                continue
            if self._elapsed_seconds(fire.verification_started_at, observed_at) > self.config.verification_timeout_sec:
                fire.state = FireState.ACTIVE
                fire.verification_valid_observations = 0
                fire.verification_started_at = None
                self._event("SUPPRESSION_VERIFICATION_TIMED_OUT", entity_id=fire.id)

    def _refresh_spatial_flags(self) -> None:
        for fire in self.fires.values():
            if self.robot.pose:
                fire.robot_within_spray_range = (
                    self.robot.pose.distance_to(fire.position)
                    <= self.config.spray_range_m
                )
            fire.threatens_person = False
            fire.threatened_person_id = None
            if (
                not self.mission
                or self.mission.status != MissionStatus.RUNNING
                or fire.state != FireState.ACTIVE
                or not self.people
            ):
                continue
            distance, person_id = min(
                (
                    fire.position.distance_to(person.position),
                    person.id,
                )
                for person in self.people.values()
            )
            if (
                distance <= self.config.person_fire_risk_distance_m
                or isclose(
                    distance,
                    self.config.person_fire_risk_distance_m,
                    abs_tol=1e-9,
                )
            ):
                fire.threatens_person = True
                fire.threatened_person_id = person_id

    def _remember_processed_result(self, action_id: str) -> None:
        self._processed_terminal_action_ids.append(action_id)
        self._processed_terminal_action_id_set.add(action_id)
        while len(self._processed_terminal_action_ids) > self.config.processed_result_cache_size:
            expired = self._processed_terminal_action_ids.popleft()
            self._processed_terminal_action_id_set.discard(expired)

    def _event(self, event_type: str, **kwargs: Any) -> None:
        self.event_log.append(Event(event_type=event_type, **kwargs))
        overflow = len(self.event_log) - self.config.max_event_log_entries
        if overflow > 0:
            del self.event_log[:overflow]

    @staticmethod
    def _lifecycle_from_result(status: ActionResultStatus) -> ActionLifecycleStatus:
        return {
            ActionResultStatus.SUCCEEDED: ActionLifecycleStatus.SUCCEEDED,
            ActionResultStatus.FAILED: ActionLifecycleStatus.FAILED,
            ActionResultStatus.ABORTED: ActionLifecycleStatus.ABORTED,
            ActionResultStatus.CANCELED: ActionLifecycleStatus.CANCELED,
            ActionResultStatus.TIMED_OUT: ActionLifecycleStatus.TIMED_OUT,
        }[status]

    @staticmethod
    def _elapsed_seconds(start_iso: str, end_iso: str) -> float:
        return (datetime.fromisoformat(end_iso) - datetime.fromisoformat(start_iso)).total_seconds()

    @staticmethod
    def _timestamp_age_seconds(timestamp_iso: str) -> float | None:
        try:
            observed_at = datetime.fromisoformat(timestamp_iso)
            if observed_at.tzinfo is None or observed_at.utcoffset() is None:
                return None
            return (utc_now() - observed_at).total_seconds()
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _serialize(value: Any) -> Any:
        if value is None:
            return None
        data = asdict(value)

        def convert(obj: Any) -> Any:
            if hasattr(obj, "value"):
                return obj.value
            if isinstance(obj, dict):
                return {k: convert(v) for k, v in obj.items()}
            if isinstance(obj, list):
                return [convert(v) for v in obj]
            return obj

        return convert(data)
