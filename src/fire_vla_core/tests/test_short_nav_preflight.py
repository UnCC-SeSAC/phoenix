from datetime import timedelta

from fire_vla_core.domain import (
    ActionDecision,
    ActionType,
    Pose2D,
    utc_now,
    utc_now_iso,
)
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.short_nav_preflight import run_short_nav_preflight
from fire_vla_core.validator import ActionValidator
from fire_vla_core.world_model import WorldModel


def navigation_action(world):
    return TargetResolver().resolve(
        ActionDecision(ActionType.RETURN_HOME, "freshness probe"),
        world,
    )


def test_continuous_robot_pose_stream_keeps_validator_fresh():
    world = WorldModel()
    world.update_robot_pose(Pose2D(0.0, 0.0, 0.0), utc_now_iso())
    world.set_mission("m1", "test")
    old_stamp = world.robot.pose_updated_at

    world.update_robot_pose(Pose2D(0.01, 0.0, 0.0), utc_now_iso())

    assert world.robot.pose_updated_at >= old_stamp
    assert ActionValidator().validate(navigation_action(world), world).approved


def test_stale_robot_pose_still_rejects():
    world = WorldModel()
    world.update_robot_pose(
        Pose2D(0.0, 0.0, 0.0),
        (utc_now() - timedelta(seconds=10)).isoformat(),
    )
    world.set_mission("m1", "test")

    result = ActionValidator().validate(navigation_action(world), world)

    assert not result.approved
    assert "너무 오래" in result.reason


def test_short_nav_software_only_e2e():
    result = run_short_nav_preflight()

    assert result["robot_pose_fresh"]
    assert result["person_ids"] == ["person_0001"]
    assert result["decision"] == "NAVIGATE_TO"
    assert result["decision_target"] == "person_0001"
    assert result["resolved_target_pose"] == {
        "x": 0.5,
        "y": 0.0,
        "yaw": 0.0,
        "frame_id": "map",
    }
    assert result["validator_approved"]
    assert result["submission"] == "ACCEPTED"
    assert result["navigation_adapter"] == "MockNavigationAdapter"
    assert result["mock_navigation_calls"] == 1
    assert result["actual_nav2_goals"] == 0
    assert result["cmd_vel_messages"] == 0
