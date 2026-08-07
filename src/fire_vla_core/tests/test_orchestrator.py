import pytest

from fire_vla_core.adapters.mock_adapters import MockNavigationAdapter, MockReportAdapter, MockResultQueue, MockSprayAdapter, MockWaitAdapter
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import ActionDecision, ActionType, Event, ObservationBatch, Pose2D, SemanticObservation, utc_now_iso
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


def test_validation_rejection_preserves_result_without_dispatch():
    _, _, orchestrator = make_orchestrator()
    decision = ActionDecision(
        ActionType.EXTINGUISH,
        "분사 범위라고 잘못 판단",
        "fire_01",
    )
    orchestrator.llm = StubLLM(decision=decision)

    cycle = orchestrator.decide_once()

    assert cycle.decision is decision
    assert cycle.validation and not cycle.validation.approved
    assert cycle.submission is None
    assert cycle.blocked_reason.startswith("ACTION_VALIDATION_REJECTED:")
    assert adapter_call_count(orchestrator) == 0


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
    ActionDecision(ActionType.EXTINGUISH, "범위 밖 화점", "fire_01"),
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


def test_inference_failure_does_not_consume_signature():
    _, _, orchestrator = make_orchestrator()
    orchestrator.llm = StubLLM(error=LLMInferenceError("device lost"))
    first = orchestrator.decide_once()
    second = orchestrator.decide_once()
    assert first.blocked_reason.startswith("LLM_INFERENCE_FAILED:")
    assert second.blocked_reason.startswith("LLM_INFERENCE_FAILED:")
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
