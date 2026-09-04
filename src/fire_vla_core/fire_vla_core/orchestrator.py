from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from math import isfinite
from typing import Any

from .dispatcher import ActionDispatcher
from .domain import (
    ActionDecision,
    ActionResultStatus,
    ActionSubmission,
    ActionSubmissionStatus,
    ActionType,
    FireState,
    MissionScope,
    utc_now,
)
from .llm import LLMError, LLMInferenceError, LLMOutputError
from .ports import ActionResultSource, LLMPort
from .resolver import TargetResolutionError, TargetResolver
from .validator import ActionValidator, ValidationResult
from .world_model import WorldModel


_POSITION_SIGNATURE_RESOLUTION_M = 0.1
_YAW_SIGNATURE_RESOLUTION_RAD = 0.1


@dataclass(frozen=True, slots=True)
class DecisionCycle:
    decision: ActionDecision | None
    validation: ValidationResult | None
    submission: ActionSubmission | None
    blocked_reason: str = ""


@dataclass(slots=True)
class VLAOrchestrator:
    world: WorldModel
    llm: LLMPort
    resolver: TargetResolver
    validator: ActionValidator
    dispatcher: ActionDispatcher
    _last_decision_input_signature: tuple[Any, ...] | None = field(
        default=None, init=False, repr=False
    )
    _semantic_action_keys: dict[str, tuple[str, ActionType, str | None]] = field(
        default_factory=dict, init=False, repr=False
    )
    _non_retryable_semantic_keys: set[tuple[str, ActionType, str | None]] = field(
        default_factory=set, init=False, repr=False
    )
    _navigation_continuations: dict[
        str, tuple[str, tuple[Any, ...]]
    ] = field(default_factory=dict, init=False, repr=False)
    _pending_continuation: tuple[str, tuple[Any, ...]] | None = field(
        default=None, init=False, repr=False
    )

    def decide_once(self) -> DecisionCycle:
        if not self.world.mission:
            return DecisionCycle(None, None, None, "Mission이 없습니다.")
        if self.world.mission.status.value != "RUNNING":
            return DecisionCycle(None, None, None, f"Mission이 {self.world.mission.status.value} 상태입니다.")

        for fire in self.world.fires.values():
            if (
                fire.state == FireState.ACTIVE
                and fire.spray_count >= self.validator.max_spray_attempts
            ):
                self.world.mark_fire_inaccessible(fire.id)
        self.world.complete_mission_if_resolved()
        if self.world.mission.status.value != "RUNNING":
            return DecisionCycle(None, None, None)

        qwen_selected_navigation = False
        if self.world.current_action is not None:
            decision = ActionDecision(ActionType.WAIT, "물리 행동이 실행 중이므로 완료 결과를 기다린다")
        else:
            decision = None
            if self._pending_continuation is not None:
                target, scene_signature = self._pending_continuation
                self._pending_continuation = None
                decision = self._build_extinguish_continuation(
                    target, scene_signature
                )
            if decision is None:
                signature = self._decision_input_signature()
                if signature == self._last_decision_input_signature:
                    return DecisionCycle(None, None, None)
                try:
                    decision = self.llm.decide(
                        self.world.mission.text,
                        self.world.create_snapshot(),
                    )
                except LLMOutputError as exc:
                    self._last_decision_input_signature = signature
                    return DecisionCycle(None, None, None, f"LLM_OUTPUT_INVALID: {exc}")
                except (LLMInferenceError, LLMError) as exc:
                    # A timed-out remote request may still be running on the server.
                    # Do not overlap it with an unchanged timer-driven retry.
                    self._last_decision_input_signature = signature
                    return DecisionCycle(None, None, None, f"LLM_INFERENCE_FAILED: {exc}")
                if decision.mission_scope is None:
                    return DecisionCycle(
                        decision, None, None,
                        "MISSION_SCOPE_INVALID: mission_scope가 필요합니다.",
                    )
                try:
                    self.world.bind_mission_scope(
                        decision.mission_scope, decision.target
                    )
                except ValueError as exc:
                    return DecisionCycle(
                        decision, None, None,
                        f"MISSION_SCOPE_INVALID: {exc}",
                    )
                self._last_decision_input_signature = signature
                qwen_selected_navigation = (
                    decision.action == ActionType.NAVIGATE_TO
                )
                decision = self._correct_out_of_range_extinguish(decision)

        if self._targets_non_mission_fire(decision):
            return DecisionCycle(
                decision,
                None,
                None,
                "MISSION_TARGET_MISMATCH: FIRE_ONLY Mission의 고정 target과 다릅니다.",
            )

        try:
            action = self.resolver.resolve(decision, self.world)
        except TargetResolutionError as exc:
            return DecisionCycle(
                decision,
                ValidationResult(False, reason=str(exc)),
                None,
                f"TARGET_RESOLUTION_FAILED: {exc}",
            )

        validation = self.validator.validate(action, self.world)
        if not validation.approved or validation.action is None:
            return DecisionCycle(
                decision,
                validation,
                None,
                f"ACTION_VALIDATION_REJECTED: {validation.reason}",
            )

        semantic_key = self._semantic_action_key(validation.action)
        if (
            semantic_key in self._non_retryable_semantic_keys
            and validation.action.action == ActionType.EXTINGUISH
            and validation.action.target in self.world.fires
            and self.world.fires[validation.action.target].state == FireState.ACTIVE
            and 0
            < self.world.fires[validation.action.target].spray_count
            < self.validator.max_spray_attempts
        ):
            self._non_retryable_semantic_keys.discard(semantic_key)
        if semantic_key in self._non_retryable_semantic_keys:
            return DecisionCycle(
                decision,
                validation,
                None,
                "DUPLICATE_ACTION_BLOCKED: 동일 Mission에서 실행 중이거나 성공한 행동입니다.",
            )

        submission = self.dispatcher.submit(validation.action)
        self.world.apply_submission(validation.action, submission)
        if submission.status == ActionSubmissionStatus.ACCEPTED:
            self._semantic_action_keys[validation.action.action_id] = semantic_key
            self._non_retryable_semantic_keys.add(semantic_key)
            if (
                qwen_selected_navigation
                and validation.action.action == ActionType.NAVIGATE_TO
                and validation.action.target in self.world.fires
            ):
                self._navigation_continuations[validation.action.action_id] = (
                    validation.action.target,
                    self._continuation_scene_signature(),
                )
        return DecisionCycle(decision, validation, submission)

    def _correct_out_of_range_extinguish(
        self, decision: ActionDecision
    ) -> ActionDecision:
        if decision.action != ActionType.EXTINGUISH or not decision.target:
            return decision
        fire = self.world.fires.get(decision.target)
        if (
            fire is None
            or fire.state != FireState.ACTIVE
            or fire.robot_within_spray_range
            or not all(
                isfinite(value)
                for value in (
                    fire.position.x,
                    fire.position.y,
                    fire.position.yaw,
                )
            )
        ):
            return decision
        return ActionDecision(
            ActionType.NAVIGATE_TO,
            "분사거리 밖 ACTIVE 화점으로 접근한다.",
            decision.target,
            decision.mission_scope,
        )

    def _targets_non_mission_fire(self, decision: ActionDecision) -> bool:
        mission = self.world.mission
        return bool(
            mission
            and mission.scope == MissionScope.FIRE_ONLY
            and mission.target_fire_id
            and decision.action in {ActionType.NAVIGATE_TO, ActionType.EXTINGUISH}
            and decision.target != mission.target_fire_id
        )

    def process_results(self, source: ActionResultSource) -> int:
        count = 0
        physical_action_completed = False
        for result in source.drain_results():
            action = self.world.pending_actions.get(result.action_id)
            continuation = self._navigation_continuations.pop(
                result.action_id, None
            )
            if action is None and self.world.current_action is not None:
                if self.world.current_action.action_id == result.action_id:
                    action = self.world.current_action
            if self.world.apply_action_result(result):
                count += 1
                physical_action_completed = physical_action_completed or bool(action and action.is_physical)
                semantic_key = self._semantic_action_keys.pop(result.action_id, None)
                if (
                    semantic_key is not None
                    and result.status
                    not in {
                        ActionResultStatus.SUCCEEDED,
                        ActionResultStatus.ABORTED,
                    }
                ):
                    self._non_retryable_semantic_keys.discard(semantic_key)
                if (
                    continuation is not None
                    and action is not None
                    and action.action == ActionType.NAVIGATE_TO
                    and result.status == ActionResultStatus.SUCCEEDED
                ):
                    self._pending_continuation = continuation
        if physical_action_completed:
            self._last_decision_input_signature = None
        self.world.complete_mission_if_resolved()
        return count

    def _build_extinguish_continuation(
        self, target: str, scene_signature: tuple[Any, ...]
    ) -> ActionDecision | None:
        if scene_signature != self._continuation_scene_signature():
            return None
        fire = self.world.fires.get(target)
        mission = self.world.mission
        robot_pose = self.world.robot.pose
        if (
            mission is None
            or (
                mission.scope == MissionScope.FIRE_ONLY
                and mission.target_fire_id != target
            )
            or fire is None
            or fire.state != FireState.ACTIVE
            or robot_pose is None
            or not self._entity_is_fresh(fire.last_seen)
            or not self._robot_pose_is_fresh_and_valid()
            or not all(isfinite(value) for value in (
                fire.position.x,
                fire.position.y,
                fire.position.yaw,
            ))
        ):
            return None
        distance = robot_pose.distance_to(fire.position)
        if distance > self.world.config.spray_range_m:
            return None
        if not fire.robot_within_spray_range:
            return None
        return ActionDecision(
            ActionType.EXTINGUISH,
            "Nav2 접근 성공 후 안전 조건을 확인해 화점을 진압한다.",
            target,
        )

    def _continuation_scene_signature(self) -> tuple[Any, ...]:
        return (
            self.world.mission.id if self.world.mission else None,
            tuple(sorted(
                (
                    person.id,
                    self._pose_signature(person.position),
                    person.state.value,
                    person.reported,
                )
                for person in self.world.people.values()
            )),
            tuple(sorted(
                (
                    fire.id,
                    self._pose_signature(fire.position),
                    fire.size,
                    fire.state.value,
                    fire.blocks_route_to,
                    fire.threatens_person,
                    fire.threatened_person_id,
                    fire.spray_count,
                )
                for fire in self.world.fires.values()
            )),
        )

    def _entity_is_fresh(self, timestamp: str) -> bool:
        try:
            observed_at = datetime.fromisoformat(timestamp)
            if observed_at.tzinfo is None or observed_at.utcoffset() is None:
                return False
            age = (utc_now() - observed_at).total_seconds()
        except (TypeError, ValueError):
            return False
        return 0.0 <= age <= self.world.config.observation_max_age_sec

    def _robot_pose_is_fresh_and_valid(self) -> bool:
        pose = self.world.robot.pose
        timestamp = self.world.robot.pose_updated_at
        if pose is None or timestamp is None:
            return False
        if not all(isfinite(value) for value in (pose.x, pose.y, pose.yaw)):
            return False
        try:
            updated_at = datetime.fromisoformat(timestamp)
            if updated_at.tzinfo is None or updated_at.utcoffset() is None:
                return False
            age = (utc_now() - updated_at).total_seconds()
        except (TypeError, ValueError):
            return False
        return 0.0 <= age <= self.world.config.robot_pose_max_age_sec

    def _semantic_action_key(
        self, action: Any
    ) -> tuple[str, ActionType, str | None]:
        assert self.world.mission is not None
        return (self.world.mission.id, action.action, action.target)

    def _decision_input_signature(self) -> tuple[Any, ...]:
        mission = self.world.mission
        robot = self.world.robot
        return (
            None if mission is None else (mission.id, mission.text, mission.status.value),
            self.world.exploration_status.value,
            self.world.perception_ready,
            (
                self._pose_signature(robot.pose),
                robot.navigation_status,
                self._pose_signature(robot.home_pose),
            ),
            tuple(sorted(
                (
                    person.id,
                    self._pose_signature(person.position),
                    person.state.value,
                    person.reported,
                )
                for person in self.world.people.values()
            )),
            tuple(sorted(
                (
                    fire.id,
                    self._pose_signature(fire.position),
                    fire.size,
                    fire.state.value,
                    fire.blocks_route_to,
                    fire.threatens_person,
                    fire.threatened_person_id,
                    fire.robot_within_spray_range,
                    fire.spray_count,
                )
                for fire in self.world.fires.values()
            )),
            tuple(sorted(self._zone_signature(zone) for zone in self.world.unexplored_zones)),
        )

    @staticmethod
    def _pose_signature(pose: Any) -> tuple[int, int, int] | None:
        if pose is None:
            return None
        values = (float(pose.x), float(pose.y), float(pose.yaw))
        if not all(isfinite(value) for value in values):
            return None
        return (
            round(values[0] / _POSITION_SIGNATURE_RESOLUTION_M),
            round(values[1] / _POSITION_SIGNATURE_RESOLUTION_M),
            round(values[2] / _YAW_SIGNATURE_RESOLUTION_RAD),
        )

    @staticmethod
    def _zone_signature(zone: dict[str, Any]) -> tuple[str, tuple[int, int, int] | None]:
        pose = zone.get("pose")
        if not isinstance(pose, dict):
            return str(zone.get("id", "")), None
        try:
            normalized_pose = (
                round(float(pose["x"]) / _POSITION_SIGNATURE_RESOLUTION_M),
                round(float(pose["y"]) / _POSITION_SIGNATURE_RESOLUTION_M),
                round(float(pose.get("yaw", 0.0)) / _YAW_SIGNATURE_RESOLUTION_RAD),
            )
        except (KeyError, TypeError, ValueError):
            normalized_pose = None
        return str(zone.get("id", "")), normalized_pose
