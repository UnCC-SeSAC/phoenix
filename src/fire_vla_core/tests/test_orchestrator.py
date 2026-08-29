import pytest

from fire_vla_core.adapters.mock_adapters import MockNavigationAdapter, MockReportAdapter, MockResultQueue, MockSprayAdapter, MockWaitAdapter
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import ActionDecision, MissionScope, ActionResult, ActionResultStatus, ActionType, Event, ExecutionSource, ObservationBatch, Pose2D, SemanticObservation, utc_now_iso
from fire_vla_core.llm import LLMInferenceError, LLMOutputError, MockVLABrain
from fire_vla_core.orchestrator import VLAOrchestrator
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.validator import ActionValidator
from fire_vla_core.world_model import WorldModel


def make_orchestrator():
    world = WorldModel()
    world.update_robot_pose(Pose2D(0, 0))
    world.set_mission("m1", "인명을 우선 확인하되 경로 차단 화점을 먼저 제거")
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_01", "person", .9, Pose2D(5, 0), now),
        SemanticObservation("fire_01", "fire", .9, Pose2D(2, 0), now, "SMALL", "person_01"),
    )))
    queue = MockResultQueue()
    dispatcher = ActionDispatcher(MockNavigationAdapter(queue), MockSprayAdapter(queue), MockReportAdapter(queue), MockWaitAdapter(queue))
    return world, queue, VLAOrchestrator(world, MockVLABrain(), TargetResolver(), ActionValidator(), dispatcher)


def test_blocking_fire_is_selected_first_and_submitted():
    world, queue, orchestrator = make_orchestrator()
    cycle = orchestrator.decide_once()
    assert cycle.validation.approved
    assert cycle.validation.action.target == "fire_01"
    assert cycle.validation.action.action == ActionType.NAVIGATE_TO
    assert world.current_action is not None
    assert orchestrator.process_results(queue) == 1
    assert world.current_action is None


def test_running_physical_action_produces_wait_decision():
    world, queue, orchestrator = make_orchestrator()
    orchestrator.decide_once()
    cycle = orchestrator.decide_once()
    assert cycle.decision.action == ActionType.WAIT


class StubLLM:
    def __init__(self, decision=None, error=None):
        self.decision = decision
        self.error = error
        self.calls = 0

    def decide(self, mission, world_model):
        self.calls += 1
        if self.error:
            raise self.error
        return self.decision


def adapter_call_count(orchestrator):
    dispatcher = orchestrator.dispatcher
    return sum(
        len(adapter.calls)
        for adapter in (
            dispatcher.navigation,
            dispatcher.spray,
            dispatcher.report,
            dispatcher.waiter,
        )
    )


def test_llm_inference_failure_is_non_dispatch_blocked_cycle():
    _, _, orchestrator = make_orchestrator()
    orchestrator.llm = StubLLM(error=LLMInferenceError("device lost"))

    cycle = orchestrator.decide_once()

    assert cycle.decision is None
    assert cycle.submission is None
    assert cycle.blocked_reason.startswith("LLM_INFERENCE_FAILED:")
    assert adapter_call_count(orchestrator) == 0


def test_parser_failure_is_non_dispatch_blocked_cycle():
    _, _, orchestrator = make_orchestrator()
    orchestrator.llm = StubLLM(error=LLMOutputError("invalid JSON"))

    cycle = orchestrator.decide_once()

    assert cycle.decision is None
    assert cycle.submission is None
    assert cycle.blocked_reason.startswith("LLM_OUTPUT_INVALID:")
    assert adapter_call_count(orchestrator) == 0


def test_resolution_failure_preserves_decision_without_dispatch():
    _, _, orchestrator = make_orchestrator()
    decision = ActionDecision(
        ActionType.NAVIGATE_TO,
        "존재하지 않는 대상으로 이동",
        "missing_01",
    )
    orchestrator.llm = StubLLM(decision=decision)

    cycle = orchestrator.decide_once()

    assert cycle.decision is decision
    assert cycle.validation and not cycle.validation.approved
    assert cycle.submission is None
    assert cycle.blocked_reason.startswith("TARGET_RESOLUTION_FAILED:")
    assert adapter_call_count(orchestrator) == 0


def test_out_of_range_active_fire_extinguish_is_corrected_to_navigation():
    _, _, orchestrator = make_orchestrator()
    decision = ActionDecision(
        ActionType.EXTINGUISH,
        "분사 범위라고 잘못 판단",
        "fire_01",
    )
    orchestrator.llm = StubLLM(decision=decision)

    cycle = orchestrator.decide_once()

    assert cycle.decision.action == ActionType.NAVIGATE_TO
    assert cycle.decision.target == "fire_01"
    assert cycle.validation and cycle.validation.approved
    assert cycle.submission is not None
    assert len(orchestrator.dispatcher.navigation.calls) == 1


def test_in_range_active_fire_keeps_extinguish_decision():
    world, _, orchestrator = make_orchestrator()
    world.update_robot_pose(Pose2D(1.3, 0))
    orchestrator.llm = StubLLM(
        ActionDecision(ActionType.EXTINGUISH, "분사 범위 안 화점 진압", "fire_01")
    )

    cycle = orchestrator.decide_once()

    assert cycle.decision.action == ActionType.EXTINGUISH
    assert cycle.validation and cycle.validation.approved
    assert len(orchestrator.dispatcher.spray.calls) == 1


def test_stale_or_invalid_fire_is_not_corrected_to_navigation():
    stale_world, _, stale_orchestrator = make_orchestrator()
    stale_world.fires.clear()
    stale_world.update_observation_batch(
        ObservationBatch(
            "2000-01-01T00:00:00+00:00",
            (
                SemanticObservation(
                    "fire_01",
                    "fire",
                    0.9,
                    Pose2D(2, 0),
                    "2000-01-01T00:00:00+00:00",
                ),
            ),
        )
    )
    stale_orchestrator.llm = StubLLM(
        ActionDecision(ActionType.EXTINGUISH, "유효하지 않은 화점", "fire_01")
    )
    stale_cycle = stale_orchestrator.decide_once()

    assert "fire_01" not in stale_world.fires
    assert stale_cycle.decision.action == ActionType.EXTINGUISH
    assert stale_cycle.blocked_reason.startswith("ACTION_VALIDATION_REJECTED:")
    assert adapter_call_count(stale_orchestrator) == 0

    invalid_world, _, invalid_orchestrator = make_orchestrator()
    invalid_world.fires["fire_01"].position = Pose2D(float("nan"), 0)
    invalid_orchestrator.llm = StubLLM(
        ActionDecision(ActionType.EXTINGUISH, "유효하지 않은 화점", "fire_01")
    )
    invalid_cycle = invalid_orchestrator.decide_once()

    assert invalid_cycle.decision.action == ActionType.EXTINGUISH
    assert invalid_cycle.validation and not invalid_cycle.validation.approved
    assert invalid_cycle.submission is None
    assert adapter_call_count(invalid_orchestrator) == 0


def test_corrected_navigation_is_not_duplicated_in_same_mission():
    world, _, orchestrator = make_orchestrator()
    orchestrator.llm = StubLLM(
        ActionDecision(ActionType.EXTINGUISH, "범위 밖 화점 진압", "fire_01")
    )

    first = orchestrator.decide_once()
    world.current_action = None
    world.pending_actions.clear()
    world.update_robot_pose(Pose2D(0.2, 0))
    second = orchestrator.decide_once()

    assert first.submission is not None
    assert second.submission is None
    assert second.blocked_reason.startswith("DUPLICATE_ACTION_BLOCKED:")
    assert len(orchestrator.dispatcher.navigation.calls) == 1


def test_normal_wait_is_dispatched_to_wait_port():
    _, _, orchestrator = make_orchestrator()
    decision = ActionDecision(ActionType.WAIT, "정상적으로 대기한다", None)
    orchestrator.llm = StubLLM(decision=decision)

    cycle = orchestrator.decide_once()

    assert cycle.decision is decision
    assert cycle.validation and cycle.validation.approved
    assert cycle.submission is not None
    assert len(orchestrator.dispatcher.waiter.calls) == 1


def test_normal_physical_action_still_uses_existing_pipeline():
    _, _, orchestrator = make_orchestrator()
    decision = ActionDecision(
        ActionType.NAVIGATE_TO,
        "화점으로 이동한다",
        "fire_01",
    )
    orchestrator.llm = StubLLM(decision=decision)

    cycle = orchestrator.decide_once()

    assert cycle.validation and cycle.validation.approved
    assert cycle.submission is not None
    assert len(orchestrator.dispatcher.navigation.calls) == 1


def wait_llm():
    return StubLLM(ActionDecision(ActionType.WAIT, "상태가 변할 때까지 대기한다", None))


def test_initial_decision_runs_once_and_unchanged_input_is_noop():
    _, _, orchestrator = make_orchestrator()
    orchestrator.llm = wait_llm()
    first = orchestrator.decide_once()
    skipped = orchestrator.decide_once()
    assert first.submission is not None
    assert orchestrator.llm.calls == 1
    assert skipped.decision is None
    assert skipped.submission is None
    assert skipped.blocked_reason == ""


def test_semantic_observation_and_mission_change_trigger_new_decisions():
    world, _, orchestrator = make_orchestrator()
    orchestrator.llm = wait_llm()
    orchestrator.decide_once()
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_02", "person", .9, Pose2D(7, 0), now),
    )))
    orchestrator.decide_once()
    assert orchestrator.llm.calls == 2
    world.set_mission("m2", "새로운 임무")
    orchestrator.decide_once()
    assert orchestrator.llm.calls == 3


def test_pose_signature_ignores_noise_but_detects_boundary_crossing():
    world, _, orchestrator = make_orchestrator()
    orchestrator.llm = wait_llm()
    orchestrator.decide_once()
    world.update_robot_pose(Pose2D(.04, .04, .04))
    orchestrator.decide_once()
    assert orchestrator.llm.calls == 1
    world.update_robot_pose(Pose2D(.11, .04, .04))
    orchestrator.decide_once()
    assert orchestrator.llm.calls == 2


def test_spray_range_change_is_decision_relevant():
    world, _, orchestrator = make_orchestrator()
    orchestrator.llm = wait_llm()
    orchestrator.decide_once()
    assert not world.fires["fire_01"].robot_within_spray_range
    world.update_robot_pose(Pose2D(1.3, 0))
    assert world.fires["fire_01"].robot_within_spray_range
    orchestrator.decide_once()
    assert orchestrator.llm.calls == 2


def test_wait_result_does_not_invalidate_decision_input():
    _, queue, orchestrator = make_orchestrator()
    orchestrator.llm = wait_llm()
    orchestrator.decide_once()
    assert orchestrator.process_results(queue) == 1
    skipped = orchestrator.decide_once()
    assert orchestrator.llm.calls == 1
    assert skipped.blocked_reason == ""


def test_physical_action_result_invalidates_decision_input():
    _, queue, orchestrator = make_orchestrator()
    orchestrator.llm = StubLLM(ActionDecision(ActionType.NAVIGATE_TO, "화점으로 이동", "fire_01"))
    orchestrator.decide_once()
    assert orchestrator.process_results(queue) == 1
    orchestrator.decide_once()
    assert orchestrator.llm.calls == 2


def test_output_blocked_cycle_is_deduplicated():
    _, _, orchestrator = make_orchestrator()
    orchestrator.llm = StubLLM(error=LLMOutputError("bad schema"))
    first = orchestrator.decide_once()
    second = orchestrator.decide_once()
    assert first.blocked_reason.startswith("LLM_OUTPUT_INVALID:")
    assert second.blocked_reason == ""
    assert orchestrator.llm.calls == 1


@pytest.mark.parametrize("decision", [
    ActionDecision(ActionType.NAVIGATE_TO, "없는 대상", "missing_01"),
])
def test_resolution_and_validation_blocked_cycles_are_deduplicated(decision):
    world, _, orchestrator = make_orchestrator()
    orchestrator.llm = StubLLM(decision)
    first = orchestrator.decide_once()
    second = orchestrator.decide_once()
    assert first.blocked_reason
    assert second.blocked_reason == ""
    assert orchestrator.llm.calls == 1
    world.set_blocks_route("fire_01", None)
    orchestrator.decide_once()
    assert orchestrator.llm.calls == 2


def test_inference_failure_blocks_unchanged_timer_retry():
    _, _, orchestrator = make_orchestrator()
    orchestrator.llm = StubLLM(error=LLMInferenceError("device lost"))
    first = orchestrator.decide_once()
    second = orchestrator.decide_once()
    assert first.blocked_reason.startswith("LLM_INFERENCE_FAILED:")
    assert second.blocked_reason == ""
    assert orchestrator.llm.calls == 1

    orchestrator.world.update_robot_pose(Pose2D(0.2, 0.0))
    third = orchestrator.decide_once()
    assert third.blocked_reason.startswith("LLM_INFERENCE_FAILED:")
    assert orchestrator.llm.calls == 2


def test_bookkeeping_changes_do_not_trigger_new_decision():
    world, queue, orchestrator = make_orchestrator()
    orchestrator.llm = wait_llm()
    orchestrator.decide_once()
    orchestrator.process_results(queue)
    world.update_robot_pose(Pose2D(0, 0))
    world.event_log.append(Event("LOG_ONLY", detail="진단 로그"))
    orchestrator.decide_once()
    assert orchestrator.llm.calls == 1


def test_running_physical_action_guard_does_not_call_llm_again():
    _, _, orchestrator = make_orchestrator()
    orchestrator.llm = StubLLM(ActionDecision(ActionType.NAVIGATE_TO, "화점으로 이동", "fire_01"))
    orchestrator.decide_once()
    cycle = orchestrator.decide_once()
    assert cycle.decision.action == ActionType.WAIT
    assert orchestrator.llm.calls == 1


def make_navigation_continuation_orchestrator():
    world = WorldModel()
    world.update_robot_pose(Pose2D(0, 0))
    world.set_mission("continuation", "화재를 진압해")
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_01", "fire", .9, Pose2D(2, 0), now),
    )))
    queue = MockResultQueue()
    navigation = MockNavigationAdapter(queue)
    spray = MockSprayAdapter(queue)
    llm = StubLLM(ActionDecision(ActionType.NAVIGATE_TO, "화점으로 접근", "fire_01"))
    dispatcher = ActionDispatcher(navigation, spray, MockReportAdapter(queue), MockWaitAdapter(queue))
    orchestrator = VLAOrchestrator(world, llm, TargetResolver(), ActionValidator(), dispatcher)
    return world, queue, navigation, spray, llm, orchestrator


def prepare_successful_navigation(world, queue, orchestrator):
    first = orchestrator.decide_once()
    action_id = first.validation.action.action_id
    world.update_robot_pose(Pose2D(1.2, 0))
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_01", "fire", .9, Pose2D(2, 0), now),
    )))
    assert orchestrator.process_results(queue) == 1
    return action_id


def test_navigation_success_continues_to_one_extinguish_without_qwen():
    world, queue, navigation, spray, llm, orchestrator = make_navigation_continuation_orchestrator()
    prepare_successful_navigation(world, queue, orchestrator)

    cycle = orchestrator.decide_once()

    assert cycle.decision.action == ActionType.EXTINGUISH
    assert cycle.decision.target == "fire_01"
    assert cycle.validation and cycle.validation.approved
    assert len(navigation.calls) == 1
    assert len(spray.calls) == 1
    assert llm.calls == 1


@pytest.mark.parametrize("invalid_state", ["resolved", "stale", "invalid"])
def test_invalid_fire_does_not_continue_to_extinguish(invalid_state):
    world, queue, _, spray, llm, orchestrator = make_navigation_continuation_orchestrator()
    orchestrator.decide_once()
    world.update_robot_pose(Pose2D(1.3, 0))
    if invalid_state == "resolved":
        world.fires["fire_01"].state = world.fires["fire_01"].state.EXTINGUISHED
    elif invalid_state == "stale":
        world.fires["fire_01"].last_seen = "2000-01-01T00:00:00+00:00"
        world.fires["fire_01"].robot_within_spray_range = True
    else:
        world.fires["fire_01"].position = Pose2D(float("nan"), 0)
        world.fires["fire_01"].robot_within_spray_range = True
    assert orchestrator.process_results(queue) == 1

    orchestrator.decide_once()

    assert len(spray.calls) == 0
    assert llm.calls == 2


@pytest.mark.parametrize("invalid_pose", ["stale", "invalid"])
def test_invalid_robot_pose_does_not_continue_to_extinguish(invalid_pose):
    world, queue, _, spray, llm, orchestrator = make_navigation_continuation_orchestrator()
    orchestrator.decide_once()
    world.update_robot_pose(Pose2D(1.3, 0))
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_01", "fire", .9, Pose2D(2, 0), now),
    )))
    if invalid_pose == "stale":
        world.robot.pose_updated_at = "2000-01-01T00:00:00+00:00"
    else:
        world.robot.pose = Pose2D(float("nan"), 0)
    assert orchestrator.process_results(queue) == 1

    orchestrator.decide_once()

    assert len(spray.calls) == 0
    assert llm.calls == 2


def test_out_of_range_after_navigation_does_not_continue_to_extinguish():
    world, queue, _, spray, llm, orchestrator = make_navigation_continuation_orchestrator()
    orchestrator.decide_once()
    world.update_robot_pose(Pose2D(1.0, 0))
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_01", "fire", .9, Pose2D(2, 0), now),
    )))
    assert orchestrator.process_results(queue) == 1

    orchestrator.decide_once()

    assert len(spray.calls) == 0
    assert llm.calls == 2


def test_meaningful_scene_change_uses_qwen_instead_of_continuation():
    world, queue, _, spray, llm, orchestrator = make_navigation_continuation_orchestrator()
    orchestrator.decide_once()
    world.update_robot_pose(Pose2D(1.3, 0))
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_01", "fire", .9, Pose2D(2, 0), now),
        SemanticObservation("person_01", "person", .9, Pose2D(2.05, 0), now),
    )))
    assert orchestrator.process_results(queue) == 1

    orchestrator.decide_once()

    assert len(spray.calls) == 0
    assert llm.calls == 2


def test_duplicate_navigation_terminal_does_not_duplicate_suppression():
    world, queue, _, spray, llm, orchestrator = make_navigation_continuation_orchestrator()
    action_id = prepare_successful_navigation(world, queue, orchestrator)
    orchestrator.decide_once()
    duplicate_source = MockResultQueue()
    duplicate_source.emit(ActionResult(
        action_id=action_id,
        source=ExecutionSource.NAVIGATION,
        status=ActionResultStatus.SUCCEEDED,
        target_id="fire_01",
    ))

    assert orchestrator.process_results(duplicate_source) == 0
    orchestrator.decide_once()

    assert len(spray.calls) == 1
    assert llm.calls == 1

def test_first_scoped_qwen_decision_is_bound_with_one_call():
    world, _, orchestrator = make_orchestrator()
    orchestrator.llm = StubLLM(ActionDecision(
        ActionType.NAVIGATE_TO,
        "대상 화점으로 이동",
        "fire_01",
        MissionScope.FIRE_ONLY,
    ))

    cycle = orchestrator.decide_once()

    assert cycle.submission is not None
    assert orchestrator.llm.calls == 1
    assert world.mission.scope == MissionScope.FIRE_ONLY
    assert world.mission.target_fire_id == "fire_01"
