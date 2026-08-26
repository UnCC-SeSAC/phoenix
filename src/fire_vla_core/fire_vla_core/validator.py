from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from math import isfinite

from .domain import utc_now

from .domain import Action, ActionLifecycleStatus, ActionType, FireState
from .world_model import WorldModel


@dataclass(frozen=True, slots=True)
class ValidationResult:
    approved: bool
    action: Action | None = None
    reason: str = ""


@dataclass(slots=True)
class ActionValidator:
    map_min_x: float = -100.0
    map_max_x: float = 100.0
    map_min_y: float = -100.0
    map_max_y: float = 100.0
    max_spray_attempts: int = 2

    def validate(self, action: Action, world: WorldModel) -> ValidationResult:
        if not action.action_id or not action.reason:
            return self._reject(action, "action_id와 reason이 필요합니다.")

        if world.current_action is not None and action.is_physical:
            return self._reject(action, "다른 물리 행동이 실행 중입니다.")

        if action.action in {ActionType.NAVIGATE_TO, ActionType.SEARCH, ActionType.RETURN_HOME}:
            if not self._robot_pose_is_fresh(world):
                return self._reject(action, "로봇 위치가 없거나 너무 오래되었습니다.")
            if action.target_pose is None:
                return self._reject(action, "이동 행동에는 유효한 목표 위치(target_pose)가 필요합니다.")
            p = action.target_pose
            if not all(isfinite(v) for v in (p.x, p.y, p.yaw)):
                return self._reject(action, "목표 위치에 유효하지 않은 숫자가 포함되어 있습니다.")
            if not (self.map_min_x <= p.x <= self.map_max_x and self.map_min_y <= p.y <= self.map_max_y):
                return self._reject(action, "목표 위치가 설정된 지도 범위를 벗어났습니다.")

        if action.action == ActionType.NAVIGATE_TO and (not action.target or not self._target_exists(action.target, world)):
            return self._reject(action, "WorldModel에 존재하지 않는 이동 대상입니다.")

        if action.action == ActionType.REPORT_PERSON:
            if not action.target or action.target not in world.people:
                return self._reject(action, "보고할 사람 대상이 WorldModel에 없습니다.")
            if world.people[action.target].reported:
                return self._reject(action, "이미 보고가 완료된 사람입니다.")

        if action.action == ActionType.EXTINGUISH:
            if not action.target or action.target not in world.fires:
                return self._reject(action, "진압할 화점 대상이 WorldModel에 없습니다.")
            fire = world.fires[action.target]
            if fire.state != FireState.ACTIVE:
                return self._reject(action, "ACTIVE 상태의 화점만 진압할 수 있습니다.")
            if not fire.robot_within_spray_range:
                return self._reject(action, "로봇이 화점의 분사 가능 범위 안에 있지 않습니다.")
            if fire.spray_count >= self.max_spray_attempts:
                return self._reject(action, "허용된 최대 분사 시도 횟수를 초과했습니다.")

        action.status = ActionLifecycleStatus.VALIDATED
        return ValidationResult(True, action=action)

    @staticmethod
    def _robot_pose_is_fresh(world: WorldModel) -> bool:
        if world.robot.pose is None or not world.robot.pose_updated_at:
            return False
        age = max(0.0, (utc_now() - datetime.fromisoformat(world.robot.pose_updated_at)).total_seconds())
        return age <= world.config.robot_pose_max_age_sec

    @staticmethod
    def _target_exists(target: str, world: WorldModel) -> bool:
        return target in world.people or target in world.fires

    @staticmethod
    def _reject(action: Action, reason: str) -> ValidationResult:
        action.status = ActionLifecycleStatus.REJECTED
        return ValidationResult(False, action=action, reason=reason)
