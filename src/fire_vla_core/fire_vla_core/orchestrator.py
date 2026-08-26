from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from .dispatcher import ActionDispatcher
from .domain import (
    ActionDecision,
    ActionResultStatus,
    ActionSubmission,
    ActionSubmissionStatus,
    ActionType,
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

    def decide_once(self) -> DecisionCycle:
        if not self.world.mission:
            return DecisionCycle(None, None, None, "Mission이 없습니다.")
        if self.world.mission.status.value != "RUNNING":
            return DecisionCycle(None, None, None, f"Mission이 {self.world.mission.status.value} 상태입니다.")

        if self.world.current_action is not None:
            decision = ActionDecision(ActionType.WAIT, "물리 행동이 실행 중이므로 완료 결과를 기다린다")
        else:
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
            self._last_decision_input_signature = signature

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
        return DecisionCycle(decision, validation, submission)

    def process_results(self, source: ActionResultSource) -> int:
        count = 0
        physical_action_completed = False
        for result in source.drain_results():
            action = self.world.pending_actions.get(result.action_id)
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
        if physical_action_completed:
            self._last_decision_input_signature = None
        self.world.complete_mission_if_resolved()
        return count

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
                (person.id, self._pose_signature(person.position), person.state.value, person.reported)
                for person in self.world.people.values()
            )),
            tuple(sorted(
                (
                    fire.id,
                    self._pose_signature(fire.position),
                    fire.size,
                    fire.state.value,
                    fire.blocks_route_to,
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
        return (
            round(float(pose.x) / _POSITION_SIGNATURE_RESOLUTION_M),
            round(float(pose.y) / _POSITION_SIGNATURE_RESOLUTION_M),
            round(float(pose.yaw) / _YAW_SIGNATURE_RESOLUTION_RAD),
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
