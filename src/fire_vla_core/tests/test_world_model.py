from datetime import timedelta

from fire_vla_core.domain import (
    Action,
    ActionResult,
    ActionResultStatus,
    ActionSubmission,
    ActionSubmissionStatus,
    ActionType,
    ExecutionSource,
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
        SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), now),
    )))
    assert world.people["person_01"].state == PersonState.DETECTED
    assert world.fires["fire_01"].state == FireState.ACTIVE
    assert world.fires["fire_01"].robot_within_spray_range is True


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
    config = WorldModelConfig(verification_required_observations=2, verification_delay_sec=0.0, verification_timeout_sec=5.0)
    world = make_world(config)
    start = utc_now()
    world.update_observation_batch(ObservationBatch(start.isoformat(), (SemanticObservation("fire_01", "fire", .9, Pose2D(.5, 0), start.isoformat()),)))
    action = Action("a1", ActionType.EXTINGUISH, "분사", target="fire_01")
    world.apply_submission(action, ActionSubmission("a1", ActionSubmissionStatus.ACCEPTED))
    world.apply_action_result(ActionResult("a1", ExecutionSource.SPRAY, ActionResultStatus.SUCCEEDED, "fire_01", timestamp=start.isoformat()))
    world.update_observation_batch(ObservationBatch((start + timedelta(seconds=1)).isoformat(), tuple()))
    assert world.fires["fire_01"].state == FireState.PENDING_VERIFICATION
    world.update_observation_batch(ObservationBatch((start + timedelta(seconds=2)).isoformat(), tuple()))
    assert world.fires["fire_01"].state == FireState.EXTINGUISHED


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


def test_empty_world_does_not_complete_mission_before_exploration_complete():
    world = make_world()
    world.perception_ready = True
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
