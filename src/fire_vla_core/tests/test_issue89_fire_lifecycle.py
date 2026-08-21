from fire_vla_core.adapters.mock_adapters import (
    MockNavigationAdapter,
    MockReportAdapter,
    MockResultQueue,
    MockSprayAdapter,
    MockWaitAdapter,
)
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import (
    ActionResultStatus,
    ActionType,
    FireState,
    ObservationBatch,
    Pose2D,
    SemanticObservation,
    utc_now_iso,
)
from fire_vla_core.llm import MockVLABrain
from fire_vla_core.orchestrator import VLAOrchestrator
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.status import VLAStatusTracker
from fire_vla_core.validator import ActionValidator
from fire_vla_core.world_model import WorldModel


def make_system(*, spray_result=ActionResultStatus.SUCCEEDED):
    world = WorldModel()
    world.update_robot_pose(Pose2D(0.0, 0.0))
    world.set_mission("issue89_software", "화재를 진압해")
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation(
            "fire_0001", "fire", 0.9, Pose2D(1.0, 0.0), now, "SMALL"
        ),
    )))
    results = MockResultQueue()
    navigation = MockNavigationAdapter(results)
    spray = MockSprayAdapter(results, next_result=spray_result)
    orchestrator = VLAOrchestrator(
        world,
        MockVLABrain(),
        TargetResolver(),
        ActionValidator(),
        ActionDispatcher(
            navigation,
            spray,
            MockReportAdapter(results),
            MockWaitAdapter(results),
        ),
    )
    return world, results, navigation, spray, orchestrator


def test_navigation_success_flows_to_single_suppression_and_ui_status():
    world, results, navigation, spray, orchestrator = make_system()
    tracker = VLAStatusTracker()

    navigation_cycle = orchestrator.decide_once()
    assert navigation_cycle.decision.action == ActionType.NAVIGATE_TO
    assert navigation_cycle.validation.approved is True
    assert orchestrator.process_results(results) == 1
    assert world.current_action is None
    assert world.robot.navigation_status == "SUCCEEDED"
    assert len(navigation.calls) == 1

    # Localization, rather than the action result, owns the robot pose.
    world.update_robot_pose(Pose2D(1.0, 0.0))
    suppression_cycle = orchestrator.decide_once()
    tracker.update(suppression_cycle)
    assert suppression_cycle.decision.action == ActionType.EXTINGUISH
    assert suppression_cycle.validation.action.target == "fire_0001"
    assert orchestrator.process_results(results) == 1

    assert len(navigation.calls) == 1
    assert len(spray.calls) == 1
    assert world.current_action is None
    assert world.last_action is spray.calls[0]
    assert world.last_action.status.value == "SUCCEEDED"
    assert world.fires["fire_0001"].state == FireState.PENDING_VERIFICATION

    status = tracker.create_payload(world.create_snapshot())
    assert status["decision"]["action"] == "EXTINGUISH"
    assert status["decision"]["target"] == "fire_0001"
    assert status["world_model"]["last_action"]["status"] == "SUCCEEDED"
    assert status["world_model"]["fires"][0]["spray_count"] == 1
    assert status["world_model"]["mission"]["status"] == "RUNNING"

    # RETURN_HOME is a valid next policy action; the completed fire goal and
    # suppression command must not be repeated.
    orchestrator.decide_once()
    assert sum(
        call.action == ActionType.NAVIGATE_TO and call.target == "fire_0001"
        for call in navigation.calls
    ) == 1
    assert len(spray.calls) == 1


def test_failed_suppression_stops_at_existing_two_attempt_limit():
    world, results, navigation, spray, orchestrator = make_system(
        spray_result=ActionResultStatus.FAILED
    )
    world.update_robot_pose(Pose2D(1.0, 0.0))

    first = orchestrator.decide_once()
    assert first.decision.action == ActionType.EXTINGUISH
    assert orchestrator.process_results(results) == 1
    assert world.current_action is None
    assert world.last_action.status.value == "FAILED"
    assert world.fires["fire_0001"].spray_count == 1

    retry = orchestrator.decide_once()
    assert retry.decision.action == ActionType.EXTINGUISH
    assert retry.submission is not None
    assert orchestrator.process_results(results) == 1
    assert world.fires["fire_0001"].spray_count == 2

    blocked = orchestrator.decide_once()
    assert blocked.submission is None
    assert "최대 분사" in blocked.validation.reason
    assert len(navigation.calls) == 0
    assert len(spray.calls) == 2
