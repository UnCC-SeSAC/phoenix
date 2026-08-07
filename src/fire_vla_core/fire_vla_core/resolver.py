from __future__ import annotations

from itertools import count

from .domain import Action, ActionDecision, ActionType, Pose2D
from .world_model import WorldModel


class TargetResolutionError(ValueError):
    pass


class TargetResolver:
    """Build an executable Action using authoritative WorldModel coordinates."""

    def __init__(self) -> None:
        self._ids = count(1)

    def resolve(self, decision: ActionDecision, world: WorldModel) -> Action:
        action_id = f"action_{next(self._ids):04d}"
        target_pose: Pose2D | None = None

        if decision.action == ActionType.NAVIGATE_TO:
            if not decision.target:
                raise TargetResolutionError("NAVIGATE_TO에는 target이 필요합니다.")
            entity = world.people.get(decision.target) or world.fires.get(decision.target)
            if entity is None:
                raise TargetResolutionError("WorldModel에 존재하지 않는 이동 대상입니다.")
            target_pose = self._pose_facing_target(world, entity.position)

        elif decision.action == ActionType.SEARCH:
            zone = world.find_unexplored_zone(decision.target)
            if zone is None:
                raise TargetResolutionError("SEARCH에 사용할 미탐색 구역이 없습니다.")
            decision = ActionDecision(decision.action, decision.reason, str(zone["id"]))
            pose = zone.get("pose")
            if not isinstance(pose, dict):
                raise TargetResolutionError("탐색 구역에 유효한 pose가 없습니다.")
            target_pose = Pose2D(float(pose["x"]), float(pose["y"]), float(pose.get("yaw", 0.0)))

        elif decision.action == ActionType.RETURN_HOME:
            if world.robot.home_pose is None:
                raise TargetResolutionError("복귀 위치(home_pose)가 없습니다.")
            target_pose = world.robot.home_pose

        return Action(
            action_id=action_id,
            action=decision.action,
            reason=decision.reason,
            target=decision.target,
            target_pose=target_pose,
        )

    @staticmethod
    def _pose_facing_target(world: WorldModel, target: Pose2D) -> Pose2D:
        yaw = world.robot.pose.yaw_to(target) if world.robot.pose else target.yaw
        return Pose2D(target.x, target.y, yaw)
