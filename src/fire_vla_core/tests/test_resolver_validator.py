from datetime import timedelta

import pytest

from fire_vla_core.domain import ActionDecision, ActionType, ObservationBatch, Pose2D, SemanticObservation, utc_now, utc_now_iso
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.validator import ActionValidator
from fire_vla_core.world_model import WorldModel


def make_world():
    world = WorldModel()
    world.update_robot_pose(Pose2D(0, 0))
    world.set_mission("m1", "인명 우선")
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_01", "person", .9, Pose2D(2, 0), now),
        SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), now),
    )))
    return world


def test_resolver_uses_authoritative_world_position():
    world = make_world()
    action = TargetResolver().resolve(ActionDecision(ActionType.NAVIGATE_TO, "이동", "person_01"), world)
    assert action.target_pose.x == 2
    assert action.target_pose.y == 0


def test_fire_navigation_goal_keeps_twenty_centimeter_standoff():
    world = make_world()
    action = TargetResolver().resolve(
        ActionDecision(ActionType.NAVIGATE_TO, "이동", "fire_01"), world
    )
    assert action.target_pose.x == pytest.approx(0.30)
    assert action.target_pose.y == pytest.approx(0.0)
    assert action.target_pose.distance_to(
        world.fires["fire_01"].position
    ) == pytest.approx(0.20)


def test_validator_rejects_new_physical_action_while_one_is_running():
    world = make_world()
    resolver = TargetResolver()
    first = resolver.resolve(ActionDecision(ActionType.NAVIGATE_TO, "첫 이동", "person_01"), world)
    world.current_action = first
    second = resolver.resolve(ActionDecision(ActionType.NAVIGATE_TO, "두 번째 이동", "fire_01"), world)
    result = ActionValidator().validate(second, world)
    assert result.approved is False
    assert "실행 중" in result.reason


def test_extinguish_requires_active_fire_in_range():
    world = make_world()
    world.update_robot_pose(Pose2D(.25, 0))
    action = TargetResolver().resolve(ActionDecision(ActionType.EXTINGUISH, "분사", "fire_01"), world)
    assert ActionValidator().validate(action, world).approved is True


def test_extinguish_rejects_fire_beyond_twenty_five_centimeters():
    world = make_world()
    world.update_robot_pose(Pose2D(.24, 0))
    action = TargetResolver().resolve(
        ActionDecision(ActionType.EXTINGUISH, "분사", "fire_01"), world
    )
    assert ActionValidator().validate(action, world).approved is False


def test_extinguish_allows_twenty_five_centimeter_boundary():
    world = make_world()
    world.update_robot_pose(Pose2D(.25, 0))
    action = TargetResolver().resolve(
        ActionDecision(ActionType.EXTINGUISH, "분사", "fire_01"), world
    )
    assert ActionValidator().validate(action, world).approved is True


def test_validator_rejects_stale_robot_pose():
    world = make_world()
    world.robot.pose_updated_at = (utc_now() - timedelta(seconds=10)).isoformat()
    action = TargetResolver().resolve(ActionDecision(ActionType.NAVIGATE_TO, "이동", "person_01"), world)
    result = ActionValidator().validate(action, world)
    assert result.approved is False
    assert "오래" in result.reason
