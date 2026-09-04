from datetime import timedelta

import pytest

from fire_vla_core.domain import (
    Action,
    ActionResult,
    ActionResultStatus,
    ActionSubmission,
    ActionSubmissionStatus,
    ActionType,
    ExecutionSource,
    MissionScope,
    FireState,
    ObservationBatch,
    PersonState,
    Pose2D,
    SemanticObservation,
    utc_now,
)
from fire_vla_core.world_model import WorldModel, WorldModelConfig


def make_world(config=None):
    world = WorldModel(config or WorldModelConfig())
    world.update_robot_pose(Pose2D(0, 0))
    world.set_mission("m1", "인명 우선")
    return world


def test_observation_creates_entities():
    world = make_world()
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_01", "person", .9, Pose2D(1, 2), now),
        SemanticObservation("fire_01", "fire", .9, Pose2D(.25, 0), now),
    )))
    assert world.people["person_01"].state == PersonState.DETECTED
    assert world.fires["fire_01"].state == FireState.ACTIVE
    assert world.fires["fire_01"].robot_within_spray_range is True


def test_new_mission_clears_entities_but_preserves_robot_and_home_pose():
    world = make_world()
    now = utc_now().isoformat()
    world.robot.home_pose = Pose2D(-1, -1)
    world.unexplored_zones = [{"id": "zone_01", "x": 2.0, "y": 3.0}]
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_01", "person", .9, Pose2D(1, 2), now),
        SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), now),
    )))
    robot_pose = world.robot.pose
    home_pose = world.robot.home_pose
    map_zones = list(world.unexplored_zones)

    world.set_mission("m2", "새 임무")

    assert world.people == {}
    assert world.fires == {}
    assert world.robot.pose == robot_pose
    assert world.robot.home_pose == home_pose
    assert world.unexplored_zones == map_zones


def test_new_mission_rejects_current_action_and_preserves_existing_state():
    world = make_world()
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_01", "person", .9, Pose2D(1, 2), now),
        SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), now),
    )))
    mission = world.mission
    people = dict(world.people)
    fires = dict(world.fires)
    action = Action("a1", ActionType.NAVIGATE_TO, "이전 이동", target="fire_01")
    world.apply_submission(
        action, ActionSubmission("a1", ActionSubmissionStatus.ACCEPTED)
    )

    with pytest.raises(ValueError, match="^MISSION_REJECTED_ACTIVE_ACTION$"):
        world.set_mission("m2", "새 임무")

    assert world.mission is mission
    assert world.current_action is action
    assert world.pending_actions == {"a1": action}
    assert world.people == people
    assert world.fires == fires


def test_new_mission_rejects_pending_action_without_current_action():
    world = make_world()
    mission = world.mission
    action = Action("a1", ActionType.WAIT, "이전 대기")
    world.apply_submission(
        action, ActionSubmission("a1", ActionSubmissionStatus.ACCEPTED)
    )
    assert world.current_action is None

    with pytest.raises(ValueError, match="^MISSION_REJECTED_ACTIVE_ACTION$"):
        world.set_mission("m2", "새 임무")

    assert world.mission is mission
    assert world.pending_actions == {"a1": action}


@pytest.mark.parametrize("distance", [0.0, 0.10])
def test_active_fire_threatens_person_at_inclusive_demo_boundary(distance):
    world = make_world(WorldModelConfig(person_fire_risk_distance_m=0.10))
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_01", "person", .9, Pose2D(1, 0), now),
        SemanticObservation(
            "fire_01", "fire", .9, Pose2D(1 + distance, 0), now
        ),
    )))

    fire = world.fires["fire_01"]
    assert fire.threatens_person is True
    assert fire.threatened_person_id == "person_01"


def test_active_fire_outside_demo_boundary_has_no_person_risk_relation():
    world = make_world(WorldModelConfig(person_fire_risk_distance_m=0.10))
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_01", "person", .9, Pose2D(1, 0), now),
        SemanticObservation("fire_01", "fire", .9, Pose2D(1.101, 0), now),
    )))

    fire = world.fires["fire_01"]
    assert fire.threatens_person is False
    assert fire.threatened_person_id is None


def test_each_active_fire_uses_nearest_person_without_changing_route_relation():
    world = make_world(WorldModelConfig(person_fire_risk_distance_m=0.10))
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_a", "person", .9, Pose2D(0, 0), now),
        SemanticObservation("person_b", "person", .9, Pose2D(1, 0), now),
        SemanticObservation(
            "fire_a", "fire", .9, Pose2D(.05, 0), now,
            blocks_route_to="person_b",
        ),
        SemanticObservation("fire_b", "fire", .9, Pose2D(.94, 0), now),
    )))

    assert world.fires["fire_a"].threatened_person_id == "person_a"
    assert world.fires["fire_b"].threatened_person_id == "person_b"
    assert world.fires["fire_a"].blocks_route_to == "person_b"


def test_terminal_result_clears_current_action_and_sets_last_action():
    world = make_world()
    action = Action("a1", ActionType.NAVIGATE_TO, "이동", target="x", target_pose=Pose2D(1, 0))
    world.apply_submission(action, ActionSubmission("a1", ActionSubmissionStatus.ACCEPTED))
    world.apply_action_result(ActionResult("a1", ExecutionSource.NAVIGATION, ActionResultStatus.SUCCEEDED, "x"))
    assert world.current_action is None
    assert world.last_action is action
    assert world.last_action.status.value == "SUCCEEDED"


def test_duplicate_result_is_ignored():
    world = make_world()
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), now),)))
    action = Action("a1", ActionType.EXTINGUISH, "분사", target="fire_01")
    world.apply_submission(action, ActionSubmission("a1", ActionSubmissionStatus.ACCEPTED))
    result = ActionResult("a1", ExecutionSource.SPRAY, ActionResultStatus.SUCCEEDED, "fire_01")
    assert world.apply_action_result(result) is True
    assert world.apply_action_result(result) is False
    assert world.fires["fire_01"].spray_count == 1


def test_fire_requires_valid_negative_observations_to_be_extinguished():
    config = WorldModelConfig(
        verification_required_observations=3,
        verification_delay_sec=0.0,
        verification_timeout_sec=5.0,
        observation_max_age_sec=10.0,
    )
    world = make_world(config)
    start = utc_now() - timedelta(seconds=4)
    world.update_observation_batch(ObservationBatch(start.isoformat(), (SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), start.isoformat()),)))
    action = Action("a1", ActionType.EXTINGUISH, "분사", target="fire_01")
    world.apply_submission(action, ActionSubmission("a1", ActionSubmissionStatus.ACCEPTED))
    world.apply_action_result(ActionResult("a1", ExecutionSource.SPRAY, ActionResultStatus.SUCCEEDED, "fire_01", timestamp=start.isoformat()))
    world.update_observation_batch(ObservationBatch((start + timedelta(seconds=1)).isoformat(), tuple()))
    assert world.fires["fire_01"].state == FireState.PENDING_VERIFICATION
    world.update_observation_batch(ObservationBatch((start + timedelta(seconds=2)).isoformat(), tuple()))
    assert world.fires["fire_01"].state == FireState.PENDING_VERIFICATION
    world.update_observation_batch(ObservationBatch((start + timedelta(seconds=3)).isoformat(), tuple()))
    assert world.fires["fire_01"].state == FireState.EXTINGUISHED


def test_fire_only_mission_completes_after_three_valid_empty_frames():
    config = WorldModelConfig(
        verification_required_observations=3,
        verification_delay_sec=0.5,
        verification_timeout_sec=5.0,
        observation_max_age_sec=10.0,
    )
    world = make_world(config)
    start = utc_now() - timedelta(seconds=4)
    world.update_observation_batch(ObservationBatch(start.isoformat(), (
        SemanticObservation(
            "fire_01", "fire", .9, Pose2D(.5, 0), start.isoformat()
        ),
    )))
    world.bind_mission_scope(MissionScope.FIRE_ONLY, "fire_01")
    action = Action("a1", ActionType.EXTINGUISH, "분사", target="fire_01")
    world.apply_submission(
        action, ActionSubmission("a1", ActionSubmissionStatus.ACCEPTED)
    )
    world.apply_action_result(ActionResult(
        "a1",
        ExecutionSource.SPRAY,
        ActionResultStatus.SUCCEEDED,
        "fire_01",
        timestamp=start.isoformat(),
    ))

    for offset in (1, 2, 3):
        world.update_observation_batch(ObservationBatch(
            (start + timedelta(seconds=offset)).isoformat(), tuple()
        ))

    assert world.fires["fire_01"].state == FireState.EXTINGUISHED
    assert world.mission.status.value == "COMPLETED"


def test_no_perception_message_does_not_verify_suppression():
    world = make_world()
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), now),
    )))
    action = Action("a1", ActionType.EXTINGUISH, "분사", target="fire_01")
    world.apply_submission(
        action, ActionSubmission("a1", ActionSubmissionStatus.ACCEPTED)
    )
    world.apply_action_result(ActionResult(
        "a1", ExecutionSource.SPRAY, ActionResultStatus.SUCCEEDED, "fire_01"
    ))

    assert world.fires["fire_01"].state == FireState.PENDING_VERIFICATION
    assert world.fires["fire_01"].verification_valid_observations == 0


def test_continued_fire_detection_returns_to_active_and_keeps_mission_running():
    world = make_world(WorldModelConfig(verification_delay_sec=0.0))
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), now),
    )))
    world.bind_mission_scope(MissionScope.FIRE_ONLY, "fire_01")
    action = Action("a1", ActionType.EXTINGUISH, "분사", target="fire_01")
    world.apply_submission(
        action, ActionSubmission("a1", ActionSubmissionStatus.ACCEPTED)
    )
    world.apply_action_result(ActionResult(
        "a1", ExecutionSource.SPRAY, ActionResultStatus.SUCCEEDED, "fire_01"
    ))
    observed_at = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(observed_at, (
        SemanticObservation(
            "fire_01", "fire", .9, Pose2D(.5, 0), observed_at
        ),
    )))

    assert world.fires["fire_01"].state == FireState.ACTIVE
    assert world.mission.status.value == "RUNNING"


def test_invalid_frame_does_not_count_as_negative_verification():
    config = WorldModelConfig(verification_required_observations=1, verification_delay_sec=0.0)
    world = make_world(config)
    start = utc_now()
    world.update_observation_batch(ObservationBatch(start.isoformat(), (SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), start.isoformat()),)))
    action = Action("a1", ActionType.EXTINGUISH, "분사", target="fire_01")
    world.apply_submission(action, ActionSubmission("a1", ActionSubmissionStatus.ACCEPTED))
    world.apply_action_result(ActionResult("a1", ExecutionSource.SPRAY, ActionResultStatus.SUCCEEDED, "fire_01", timestamp=start.isoformat()))
    world.update_observation_batch(ObservationBatch((start + timedelta(seconds=1)).isoformat(), tuple(), frame_valid=False))
    assert world.fires["fire_01"].state == FireState.PENDING_VERIFICATION


def test_stale_frame_does_not_count_as_negative_verification():
    config = WorldModelConfig(
        verification_required_observations=1,
        verification_delay_sec=0.0,
        observation_max_age_sec=0.1,
    )
    world = make_world(config)
    now = utc_now()
    world.update_observation_batch(ObservationBatch(now.isoformat(), (
        SemanticObservation(
            "fire_01", "fire", .9, Pose2D(.5, 0), now.isoformat()
        ),
    )))
    action = Action("a1", ActionType.EXTINGUISH, "분사", target="fire_01")
    world.apply_submission(
        action, ActionSubmission("a1", ActionSubmissionStatus.ACCEPTED)
    )
    world.apply_action_result(ActionResult(
        "a1", ExecutionSource.SPRAY, ActionResultStatus.SUCCEEDED, "fire_01"
    ))
    stale = (utc_now() - timedelta(seconds=1)).isoformat()
    world.update_observation_batch(ObservationBatch(stale, tuple()))

    assert world.fires["fire_01"].state == FireState.PENDING_VERIFICATION
    assert world.fires["fire_01"].verification_valid_observations == 0


def test_empty_world_does_not_complete_mission_before_exploration_complete():
    world = make_world()
    world.perception_ready = True
    world.mission.scope = MissionScope.FULL_EXPLORATION
    assert world.mission_goals_resolved() is False
    world.mark_exploration_completed()
    assert world.mission_goals_resolved() is True


def test_stale_observation_is_ignored():
    config = WorldModelConfig(observation_max_age_sec=0.1)
    world = make_world(config)
    old = (utc_now() - timedelta(seconds=10)).isoformat()
    world.update_observation_batch(ObservationBatch(old, (SemanticObservation("person_old", "person", .9, Pose2D(1, 1), old),)))
    assert "person_old" not in world.people
    assert any(e.event_type == "STALE_OBSERVATION_IGNORED" for e in world.event_log)

def test_fire_only_scope_ignores_unrelated_entities_and_exploration():
    world = make_world()
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_other", "person", .9, Pose2D(4, 0), now),
        SemanticObservation("fire_target", "fire", .9, Pose2D(.5, 0), now),
        SemanticObservation("fire_other", "fire", .9, Pose2D(3, 0), now),
    )))
    world.bind_mission_scope(MissionScope.FIRE_ONLY, "fire_target")
    world.fires["fire_target"].state = FireState.EXTINGUISHED

    assert world.mission_goals_resolved() is True


def test_person_fire_scope_uses_bound_relation_only():
    world = make_world()
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_target", "person", .9, Pose2D(.55, 0), now),
        SemanticObservation("person_other", "person", .9, Pose2D(4, 0), now),
        SemanticObservation("fire_target", "fire", .9, Pose2D(.5, 0), now),
        SemanticObservation("fire_other", "fire", .9, Pose2D(3, 0), now),
    )))
    world.bind_mission_scope(MissionScope.PERSON_FIRE, "fire_target")
    world.people["person_target"].reported = True
    world.people["person_target"].state = PersonState.REPORTED
    world.fires["fire_target"].state = FireState.EXTINGUISHED

    assert world.mission.target_person_id == "person_target"
    assert world.mission_goals_resolved() is True


def test_full_exploration_scope_keeps_global_completion_contract():
    world = make_world()
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_01", "person", .9, Pose2D(1, 0), now),
        SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), now),
    )))
    world.bind_mission_scope(MissionScope.FULL_EXPLORATION, None)
    assert world.mission_goals_resolved() is False
    world.mark_exploration_completed()
    assert world.mission_goals_resolved() is False
    world.people["person_01"].reported = True
    world.people["person_01"].state = PersonState.REPORTED
    world.fires["fire_01"].state = FireState.EXTINGUISHED

    assert world.mission_goals_resolved() is True


def test_mission_scope_cannot_change_after_first_binding():
    world = make_world()
    now = utc_now().isoformat()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), now),
    )))
    world.bind_mission_scope(MissionScope.FIRE_ONLY, "fire_01")

    with pytest.raises(ValueError, match="변경"):
        world.bind_mission_scope(MissionScope.FULL_EXPLORATION, None)
