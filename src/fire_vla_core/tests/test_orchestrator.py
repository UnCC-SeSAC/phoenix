from datetime import timedelta

import pytest

from fire_vla_core.adapters.mock_adapters import MockNavigationAdapter, MockReportAdapter, MockResultQueue, MockSprayAdapter, MockWaitAdapter
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import ActionDecision, MissionScope, ActionResult, ActionResultStatus, ActionType, Event, ExecutionSource, FireState, ObservationBatch, PersonState, Pose2D, SemanticObservation, utc_now_iso
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


def test_new_mission_waits_for_first_valid_post_boundary_observation():
    world = WorldModel()
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_old", "fire", .9, Pose2D(1, 0), now),
    )))
    world.set_mission("m_fresh", "현장의 불을 모두 꺼줘")
    queue = MockResultQueue()
    llm = StubLLM(ActionDecision(
        ActionType.RETURN_HOME,
        "관측 없이 잘못 복귀",
        mission_scope=MissionScope.FULL_EXPLORATION,
    ))
    orchestrator = VLAOrchestrator(
        world,
        llm,
        TargetResolver(),
        ActionValidator(),
        ActionDispatcher(
            MockNavigationAdapter(queue),
            MockSprayAdapter(queue),
            MockReportAdapter(queue),
            MockWaitAdapter(queue),
        ),
    )

    waiting = orchestrator.decide_once()

    assert waiting.blocked_reason == "MISSION_WAITING_FOR_FRESH_OBSERVATION"
    assert llm.calls == 0
    assert world.fires == {}

    observed_at = utc_now_iso()
    world.update_observation_batch(ObservationBatch(observed_at, (
        SemanticObservation(
            "fire_new", "fire", .9, Pose2D(1, 0), observed_at
        ),
    )))

    orchestrator.decide_once()
    assert llm.calls == 1
    assert set(world.fires) == {"fire_new"}


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
    world, _, orchestrator = make_orchestrator()
    world.fires["fire_01"].position = Pose2D(.5, 0)
    world.update_robot_pose(Pose2D(.19, 0))
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
    world.fires["fire_01"].position = Pose2D(.5, 0)
    world.update_robot_pose(Pose2D(.20, 0))
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


def test_residual_fire_outside_range_allows_one_reapproach_per_spray_attempt():
    world, queue, orchestrator = make_orchestrator()

    first_navigation = orchestrator.decide_once()
    assert first_navigation.validation.action.action == ActionType.NAVIGATE_TO
    assert orchestrator.process_results(queue) == 1

    fire = world.fires["fire_01"]
    fire.spray_count = 1
    fire.state = FireState.ACTIVE
    world.update_robot_pose(Pose2D(0.2, 0))

    reapproach = orchestrator.decide_once()
    assert reapproach.validation.action.action == ActionType.NAVIGATE_TO
    assert reapproach.validation.action.target == "fire_01"
    assert reapproach.submission is not None
    assert orchestrator.process_results(queue) == 1

    duplicate = orchestrator.decide_once()
    assert duplicate.submission is None
    assert duplicate.blocked_reason.startswith("DUPLICATE_ACTION_BLOCKED:")
    assert len(orchestrator.dispatcher.navigation.calls) == 2


def make_person_two_fire_orchestrator(decision):
    world = WorldModel()
    world.update_robot_pose(Pose2D(0, 0))
    world.set_mission(
        "mission_ab",
        "사람 가까이에 있는 화재를 우선 진압하고 나머지 잔불도 전부 꺼줘.",
    )
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_a", "person", .9, Pose2D(2, 0), now),
        SemanticObservation("fire_b", "fire", .9, Pose2D(4, 0), now),
        SemanticObservation("fire_a", "fire", .9, Pose2D(2.05, 0), now),
    )))
    queue = MockResultQueue()
    navigation = MockNavigationAdapter(queue)
    dispatcher = ActionDispatcher(
        navigation,
        MockSprayAdapter(queue),
        MockReportAdapter(queue),
        MockWaitAdapter(queue),
    )
    orchestrator = VLAOrchestrator(
        world,
        StubLLM(decision),
        TargetResolver(),
        ActionValidator(),
        dispatcher,
    )
    return world, navigation, orchestrator


def test_ab_demo_mission_boundary_cannot_send_frontier_before_person_returns():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(ActionType.SEARCH, "Unexplored zone requires exploration before handling fires",
                       "frontier_-6_15", MissionScope.FULL_EXPLORATION)
    )
    world.config.ab_demo_mode = True
    world.unexplored_zones = [{
        "id": "frontier_-6_15", "pose": {"x": -.649, "y": 1.453},
    }]
    world.set_mission("new_ab", world.mission.text)
    stamp = utc_now_iso()
    world.update_observation_batch(ObservationBatch(stamp, (
        SemanticObservation("fresh_a", "fire", .9, Pose2D(2.05, 0), stamp),
        SemanticObservation("fresh_b", "fire", .9, Pose2D(4, 0), stamp),
    )))
    assert not world.people
    assert not world.fire_search_hints
    rejected = orchestrator.decide_once()
    assert rejected.submission is None
    assert rejected.blocked_reason.startswith("TARGET_RESOLUTION_FAILED:")
    assert not navigation.calls
    stamp = utc_now_iso()
    world.update_observation_batch(ObservationBatch(stamp, (
        SemanticObservation("fresh_person", "person", .9, Pose2D(2, 0), stamp),
    )))
    approved = orchestrator.decide_once()
    assert approved.decision.action == ActionType.NAVIGATE_TO
    assert approved.decision.target == "fresh_a"
    assert [a.target for a in navigation.calls] == ["fresh_a"]


@pytest.mark.parametrize(
    "proposed_action",
    (ActionType.NAVIGATE_TO, ActionType.SEARCH, ActionType.RETURN_HOME),
)
def test_full_exploration_prioritizes_person_threatening_fire(proposed_action):
    target = "fire_b" if proposed_action == ActionType.NAVIGATE_TO else None
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            proposed_action,
            "다른 행동을 제안",
            target,
            MissionScope.FULL_EXPLORATION,
        )
    )

    cycle = orchestrator.decide_once()

    assert world.fires["fire_a"].threatens_person is True
    assert world.fires["fire_b"].threatens_person is False
    assert cycle.decision.action == ActionType.NAVIGATE_TO
    assert cycle.decision.target == "fire_a"
    assert cycle.submission is not None
    assert navigation.calls[-1].target == "fire_a"


@pytest.mark.parametrize(
    "resolved_state",
    (FireState.EXTINGUISHED, FireState.SUPPRESSION_FAILED),
)
def test_full_exploration_continues_to_other_fire_after_priority_fire_resolved(
    resolved_state,
):
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.NAVIGATE_TO,
            "남은 일반 화재 처리",
            "fire_b",
            MissionScope.FULL_EXPLORATION,
        )
    )
    world.fires["fire_a"].state = resolved_state

    cycle = orchestrator.decide_once()

    assert cycle.decision.target == "fire_b"
    assert cycle.submission is not None
    assert navigation.calls[-1].target == "fire_b"


def test_bound_mission_scope_overrides_later_qwen_scope_change():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.NAVIGATE_TO,
            "사람 위협 화재로 이동",
            "fire_a",
            MissionScope.PERSON_FIRE,
        )
    )
    world.bind_mission_scope(MissionScope.FULL_EXPLORATION, None)

    cycle = orchestrator.decide_once()

    assert cycle.blocked_reason == ""
    assert cycle.decision.mission_scope == MissionScope.FULL_EXPLORATION
    assert world.mission.scope == MissionScope.FULL_EXPLORATION
    assert cycle.submission is not None
    assert navigation.calls[-1].target == "fire_a"


def test_full_exploration_aligns_to_new_in_range_fire_before_extinguish():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.NAVIGATE_TO,
            "사람 위협 화재 접근",
            "fire_a",
            MissionScope.FULL_EXPLORATION,
        )
    )
    first = orchestrator.decide_once()
    assert first.decision.target == "fire_a"
    assert orchestrator.process_results(navigation.result_queue) == 1
    world.fires["fire_a"].state = FireState.EXTINGUISHED
    world.fires["fire_b"].position = Pose2D(.25, .10)
    world.update_robot_pose(Pose2D(0, 0))
    orchestrator.llm = StubLLM(ActionDecision(
        ActionType.EXTINGUISH,
        "거리 안의 남은 일반 화재 처리",
        "fire_b",
        MissionScope.FULL_EXPLORATION,
    ))

    align = orchestrator.decide_once()

    assert world.fires["fire_b"].robot_within_spray_range is True
    assert align.decision.action == ActionType.NAVIGATE_TO
    assert align.decision.target == "fire_b"
    assert align.submission is not None
    assert navigation.calls[-1].target == "fire_b"


def test_full_exploration_extinguishes_same_target_after_alignment():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.NAVIGATE_TO,
            "사람 위협 화재 접근",
            "fire_a",
            MissionScope.FULL_EXPLORATION,
        )
    )
    first = orchestrator.decide_once()
    assert first.decision.target == "fire_a"
    assert orchestrator.process_results(navigation.result_queue) == 1
    world.fires["fire_a"].state = FireState.EXTINGUISHED
    world.fires["fire_b"].position = Pose2D(.25, .10)
    world.update_robot_pose(Pose2D(0, 0))
    orchestrator.llm = StubLLM(ActionDecision(
        ActionType.EXTINGUISH,
        "거리 안의 남은 일반 화재 처리",
        "fire_b",
        MissionScope.FULL_EXPLORATION,
    ))

    align = orchestrator.decide_once()
    assert align.decision.action == ActionType.NAVIGATE_TO
    assert orchestrator.process_results(navigation.result_queue) == 1

    extinguish = orchestrator.decide_once()

    assert extinguish.decision.action == ActionType.EXTINGUISH
    assert extinguish.decision.target == "fire_b"
    assert extinguish.submission is not None


def test_full_exploration_allows_one_realign_to_previously_visited_fire():
    world = WorldModel()
    world.update_robot_pose(Pose2D(0, 0))
    world.set_mission("mission_realign", "모든 화재를 진압해")
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_a", "fire", .9, Pose2D(.2, .1), now),
        SemanticObservation("fire_b", "fire", .9, Pose2D(.2, -.1), now),
    )))
    queue = MockResultQueue()
    navigation = MockNavigationAdapter(queue)
    llm = StubLLM(ActionDecision(
        ActionType.NAVIGATE_TO,
        "B 화점 정렬",
        "fire_b",
        MissionScope.FULL_EXPLORATION,
    ))
    orchestrator = VLAOrchestrator(
        world,
        llm,
        TargetResolver(),
        ActionValidator(),
        ActionDispatcher(
            navigation,
            MockSprayAdapter(queue),
            MockReportAdapter(queue),
            MockWaitAdapter(queue),
        ),
    )

    first_b = orchestrator.decide_once()
    assert first_b.submission is not None
    assert orchestrator.process_results(queue) == 1
    orchestrator._pending_continuation = None

    llm.decision = ActionDecision(
        ActionType.NAVIGATE_TO,
        "A 화점 정렬",
        "fire_a",
        MissionScope.FULL_EXPLORATION,
    )
    align_a = orchestrator.decide_once()
    assert align_a.submission is not None
    assert orchestrator.process_results(queue) == 1
    orchestrator._pending_continuation = None

    llm.decision = ActionDecision(
        ActionType.EXTINGUISH,
        "B 화점으로 돌아가 진압",
        "fire_b",
        MissionScope.FULL_EXPLORATION,
    )
    realign_b = orchestrator.decide_once()

    assert realign_b.blocked_reason == ""
    assert realign_b.decision.action == ActionType.NAVIGATE_TO
    assert realign_b.decision.target == "fire_b"
    assert realign_b.submission is not None
    assert [call.target for call in navigation.calls] == [
        "fire_b",
        "fire_a",
        "fire_b",
    ]

    assert orchestrator.process_results(queue) == 1
    orchestrator._aligned_fire_target = (world.mission.id, "fire_a")
    orchestrator._pending_continuation = None

    # The bounded exception is consumed; even if alignment later changes
    # again, the same B navigation cannot be submitted a third time.
    duplicate = orchestrator.decide_once()
    assert duplicate.submission is None
    assert duplicate.blocked_reason.startswith("DUPLICATE_ACTION_BLOCKED:")
    assert len(navigation.calls) == 3


def test_full_exploration_does_not_renavigate_aligned_fire_after_pose_jitter():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.NAVIGATE_TO,
            "사람 위협 화재 접근",
            "fire_a",
            MissionScope.FULL_EXPLORATION,
        )
    )
    orchestrator.decide_once()
    assert orchestrator.process_results(navigation.result_queue) == 1
    world.fires["fire_a"].state = FireState.SUPPRESSION_FAILED
    world.fires["fire_b"].position = Pose2D(.25, .10)
    world.update_robot_pose(Pose2D(0, 0))
    orchestrator.llm = StubLLM(ActionDecision(
        ActionType.EXTINGUISH,
        "거리 안의 남은 일반 화재 처리",
        "fire_b",
        MissionScope.FULL_EXPLORATION,
    ))

    align = orchestrator.decide_once()
    assert align.decision.action == ActionType.NAVIGATE_TO
    assert orchestrator.process_results(navigation.result_queue) == 1

    # The same static fire can jitter across the 0.1 m observation signature
    # boundary after Nav2 succeeds.  That must not create a duplicate goal.
    world.fires["fire_b"].position = Pose2D(.31, .10)
    world.update_robot_pose(Pose2D(.06, 0))

    extinguish = orchestrator.decide_once()

    assert extinguish.decision.action == ActionType.EXTINGUISH
    assert extinguish.decision.target == "fire_b"
    assert extinguish.submission is not None
    assert len(navigation.calls) == 2


def test_full_exploration_realigns_once_after_material_fire_position_change():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.NAVIGATE_TO,
            "사람 위협 화재 접근",
            "fire_a",
            MissionScope.FULL_EXPLORATION,
        )
    )
    orchestrator.decide_once()
    assert orchestrator.process_results(navigation.result_queue) == 1
    orchestrator._pending_continuation = None

    world.fires["fire_a"].position = Pose2D(.33, .1)
    world.update_robot_pose(Pose2D(.08, 0))
    orchestrator.llm = StubLLM(ActionDecision(
        ActionType.EXTINGUISH,
        "fresh 좌표의 화재 진압",
        "fire_a",
        MissionScope.FULL_EXPLORATION,
    ))

    realign = orchestrator.decide_once()

    assert realign.decision.action == ActionType.NAVIGATE_TO
    assert realign.decision.target == "fire_a"
    assert realign.submission is not None
    assert len(navigation.calls) == 2


def test_fire_position_change_during_navigation_requires_realign_before_spray():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(ActionType.NAVIGATE_TO, "A 접근", "fire_a", MissionScope.FULL_EXPLORATION)
    )
    orchestrator.decide_once()
    submitted_position = world.fires["fire_a"].position
    world.fires["fire_a"].position = Pose2D(2.05, .13)
    world.update_robot_pose(Pose2D(1.9, 0))
    orchestrator.process_results(navigation.result_queue)
    assert orchestrator._aligned_fire_position == submitted_position
    orchestrator.llm = StubLLM(ActionDecision(
        ActionType.EXTINGUISH, "fresh A 진압", "fire_a", MissionScope.FULL_EXPLORATION
    ))

    cycle = orchestrator.decide_once()

    assert cycle.decision.action == ActionType.NAVIGATE_TO
    assert cycle.submission is not None
    assert len(navigation.calls) == 2
    assert orchestrator.dispatcher.spray.calls == []


@pytest.mark.parametrize("age", [6, 22])
def test_direct_extinguish_uses_stored_active_target_without_new_age_gate(age):
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(ActionType.EXTINGUISH, "직접 A 진압", "fire_a", MissionScope.FULL_EXPLORATION)
    )
    world.update_robot_pose(Pose2D(1.9, 0))
    from fire_vla_core.domain import utc_now
    world.fires["fire_a"].last_seen = (
        (utc_now() - timedelta(seconds=age)).isoformat()
    )

    accepted = orchestrator.decide_once()
    assert accepted.validation is not None and accepted.validation.approved
    assert accepted.submission is not None
    assert len(orchestrator.dispatcher.spray.calls) == 1


def test_full_exploration_retargets_stale_qwen_choice_to_active_fire():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.EXTINGUISH,
            "이미 종료된 A 화재를 다시 선택",
            "fire_a",
            MissionScope.FULL_EXPLORATION,
        )
    )
    world.fires["fire_a"].state = FireState.EXTINGUISHED

    cycle = orchestrator.decide_once()

    assert cycle.decision.action == ActionType.NAVIGATE_TO
    assert cycle.decision.target == "fire_b"
    assert cycle.submission is not None
    assert navigation.calls[-1].target == "fire_b"


@pytest.mark.parametrize("b_reobserved", [False, True])
def test_mission_boundary_search_preserves_person_threat_priority(b_reobserved):
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(ActionType.SEARCH, "후보 재확인", "fire_hint_fire_b",
                       MissionScope.FULL_EXPLORATION)
    )
    assert world.fires["fire_a"].threatens_person
    # On the first Mission there is no running Mission to populate spatial flags.
    world.mission = None
    world._refresh_spatial_flags()
    assert world.fires["fire_a"].threatens_person is False
    # Reproduce B being inserted before A in the pre-Mission observation history.
    world.fires = {"fire_b": world.fires["fire_b"], "fire_a": world.fires["fire_a"]}
    world.set_mission("new_ab", "사람 근처 화재 먼저 나머지도 진압")
    assert world.fire_search_hints[1]["threatens_person"] is True
    world.perception_ready = True
    if b_reobserved:
        now = utc_now_iso()
        world.update_observation_batch(ObservationBatch(now, (
            SemanticObservation("fresh_b", "fire", .9, Pose2D(-1.9, 0), now),
        )))
    cycle = orchestrator.decide_once()
    assert cycle.submission is not None
    assert navigation.calls[-1].action == ActionType.SEARCH
    assert navigation.calls[-1].target == "fire_hint_fire_a"
    assert world.fires.get("fire_a") is None  # Hint is not an ACTIVE fire.


def test_full_exploration_retargets_terminal_fire_to_saved_fire_hint():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.EXTINGUISH,
            "이미 종료된 A 화재를 다시 선택",
            "fire_a",
            MissionScope.FULL_EXPLORATION,
        )
    )
    world.fires["fire_a"].state = FireState.SUPPRESSION_FAILED
    world.fires["fire_b"].state = FireState.SUPPRESSION_FAILED
    world.bounded_fire_search = True
    world.fire_search_hints = [{
        "id": "fire_hint_fire_b",
        "source_fire_id": "fire_b",
        "pose": {"x": -1.9, "y": 0.0, "yaw": 3.14},
    }]

    cycle = orchestrator.decide_once()

    assert cycle.decision.action == ActionType.SEARCH
    assert cycle.decision.target == "fire_hint_fire_b"
    assert cycle.submission is not None
    assert navigation.calls[-1].action == ActionType.SEARCH
    assert navigation.calls[-1].target == "fire_hint_fire_b"


def test_bounded_fire_search_then_returns_home_after_fresh_empty_observation():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.EXTINGUISH,
            "이미 종료된 A 화재를 다시 선택",
            "fire_a",
            MissionScope.FULL_EXPLORATION,
        )
    )
    world.people["person_a"].reported = True
    world.people["person_a"].state = PersonState.REPORTED
    world.fires["fire_a"].state = FireState.SUPPRESSION_FAILED
    world.fires["fire_b"].state = FireState.SUPPRESSION_FAILED
    world.bounded_fire_search = True
    world.fire_search_hints = [{
        "id": "fire_hint_fire_b",
        "source_fire_id": "fire_b",
        "pose": {"x": -1.9, "y": 0.0, "yaw": 3.14},
    }]

    search = orchestrator.decide_once()
    assert search.decision.action == ActionType.SEARCH
    assert orchestrator.process_results(navigation.result_queue) == 1
    assert world.perception_ready is False

    observed_at = utc_now_iso()
    world.update_observation_batch(ObservationBatch(observed_at, ()))
    returning = orchestrator.decide_once()
    assert returning.decision.action == ActionType.RETURN_HOME
    assert returning.submission is not None
    assert orchestrator.process_results(navigation.result_queue) == 1

    assert world.mission.status.value == "COMPLETED_WITH_ESCALATION"
    assert [call.action for call in navigation.calls] == [
        ActionType.SEARCH,
        ActionType.RETURN_HOME,
    ]


def test_bounded_fire_search_does_not_dispatch_generic_frontier():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.SEARCH,
            "일반 frontier 탐색 제안",
            "frontier_12_28",
            MissionScope.FULL_EXPLORATION,
        )
    )
    world.fires["fire_a"].state = FireState.SUPPRESSION_FAILED
    world.fires["fire_b"].state = FireState.SUPPRESSION_FAILED
    world.bounded_fire_search = True
    world.fire_search_hints = [{
        "id": "fire_hint_fire_b",
        "source_fire_id": "fire_b",
        "pose": {"x": 0.8, "y": -0.2, "yaw": 0.0},
    }]
    world.unexplored_zones = [{
        "id": "frontier_12_28",
        "pose": {"x": 1.2, "y": 2.8, "yaw": 0.8},
    }]

    cycle = orchestrator.decide_once()

    assert cycle.decision.action == ActionType.SEARCH
    assert cycle.decision.target == "fire_hint_fire_b"
    assert navigation.calls[-1].target == "fire_hint_fire_b"


@pytest.mark.parametrize("next_target", ["fire_a", "fire_b"])
def test_hint_consumed_during_search_still_requires_target_alignment(next_target):
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(ActionType.SEARCH, "A 후보 확인", "fire_hint_fire_a",
                       MissionScope.FULL_EXPLORATION)
    )
    world.people.clear()
    world.fires.clear()
    world.config.spray_range_m = .5
    world.bounded_fire_search = True
    world.fire_search_hints = [{
        "id": "fire_hint_fire_a", "source_fire_id": "fire_a",
        "pose": {"x": .8, "y": .2, "yaw": 0.0},
    }]
    search = orchestrator.decide_once()
    assert search.submission is not None
    # A/B reappear while SEARCH is still executing, consuming the hint early.
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_a", "fire", .9, Pose2D(.8, .2), now),
        SemanticObservation("fire_b", "fire", .9, Pose2D(.8, -.2), now),
    )))
    assert not world.fire_search_hints
    world.update_robot_pose(Pose2D(.55, .2))
    assert all(f.robot_within_spray_range for f in world.fires.values())
    assert orchestrator.process_results(navigation.result_queue) == 1
    if next_target == "fire_b":
        world.fires["fire_a"].state = FireState.SUPPRESSION_FAILED
        world.fires["fire_a"].spray_count = 3
    orchestrator.llm = StubLLM(ActionDecision(
        ActionType.EXTINGUISH, "선택 화점 진압", next_target,
        MissionScope.FULL_EXPLORATION,
    ))
    aligned = orchestrator.decide_once()
    assert aligned.decision.action == ActionType.NAVIGATE_TO
    assert aligned.decision.target == next_target
    assert aligned.submission is not None
    assert orchestrator.dispatcher.spray.calls == []
    assert orchestrator.process_results(navigation.result_queue) == 1
    spray = orchestrator.decide_once()
    assert spray.decision.action == ActionType.EXTINGUISH
    assert spray.decision.target == next_target
    assert len(orchestrator.dispatcher.spray.calls) == 1
    assert [call.target for call in navigation.calls] == ["fire_hint_fire_a", next_target]


@pytest.mark.parametrize("reobserved", [True, False])
def test_bounded_fire_search_reacquires_missing_b_fire_after_mission_boundary(reobserved):
    world = WorldModel()
    world.update_robot_pose(Pose2D(0, 0))
    before = utc_now_iso()
    world.update_observation_batch(ObservationBatch(before, (
        SemanticObservation("fire_a", "fire", .9, Pose2D(.8, .2), before),
        SemanticObservation("fire_b", "fire", .9, Pose2D(.8, -.2), before),
    )))
    world.set_mission("mission_ab", "사람 근처 화재부터 나머지 화재까지 진압")
    after = utc_now_iso()
    world.update_observation_batch(ObservationBatch(after, (
        SemanticObservation("fire_a", "fire", .9, Pose2D(.8, .2), after),
    )))
    world.fires["fire_a"].state = FireState.SUPPRESSION_FAILED

    queue = MockResultQueue()
    navigation = MockNavigationAdapter(queue)
    llm = StubLLM(ActionDecision(
        ActionType.EXTINGUISH,
        "종료된 A를 다시 선택",
        "fire_a",
        MissionScope.FULL_EXPLORATION,
    ))
    orchestrator = VLAOrchestrator(
        world,
        llm,
        TargetResolver(),
        ActionValidator(),
        ActionDispatcher(
            navigation,
            MockSprayAdapter(queue),
            MockReportAdapter(queue),
            MockWaitAdapter(queue),
        ),
    )

    search = orchestrator.decide_once()
    assert search.decision.action == ActionType.SEARCH
    assert search.decision.target == "fire_hint_fire_b"
    hint_pose = Pose2D(.8, -.2)
    search_pose = search.validation.action.target_pose
    assert search_pose is not None
    assert search_pose.distance_to(hint_pose) == pytest.approx(
        world.config.navigation_standoff_m
    )
    assert search_pose.yaw == pytest.approx(search_pose.yaw_to(hint_pose))
    assert orchestrator.process_results(queue) == 1
    assert world.perception_ready is False

    if not reobserved:
        waiting = orchestrator.decide_once()
        assert waiting.blocked_reason == "MISSION_WAITING_FOR_FRESH_OBSERVATION"
        assert len(navigation.calls) == 1
        assert orchestrator.dispatcher.spray.calls == []
        # A valid fresh empty frame, rather than silence, ends the one-time hint visit.
        empty_at = utc_now_iso()
        world.update_observation_batch(ObservationBatch(empty_at, ()))
        world.update_robot_pose(search_pose)
        returning = orchestrator.decide_once()
        assert returning.decision.action == ActionType.RETURN_HOME
        assert len(navigation.calls) == 2
        assert orchestrator.dispatcher.spray.calls == []
        return

    reacquired_at = utc_now_iso()
    world.update_observation_batch(ObservationBatch(reacquired_at, (
        SemanticObservation(
            "fire_b", "fire", .9, Pose2D(.81, -.2), reacquired_at
        ),
    )))
    llm.decision = ActionDecision(
        ActionType.NAVIGATE_TO,
        "재관측한 B 화재로 이동",
        "fire_b",
        MissionScope.FULL_EXPLORATION,
    )

    reacquired = orchestrator.decide_once()
    assert reacquired.decision.action == ActionType.NAVIGATE_TO
    assert reacquired.decision.target == "fire_b"
    assert reacquired.submission is not None
    assert [call.target for call in navigation.calls] == [
        "fire_hint_fire_b",
        "fire_b",
    ]


def test_full_exploration_searches_frontier_then_accepts_new_fire():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.SEARCH,
            "남은 미탐색 구역을 확인",
            "frontier_30_0",
            MissionScope.FULL_EXPLORATION,
        )
    )
    world.fires["fire_a"].state = FireState.EXTINGUISHED
    world.fires.pop("fire_b")
    world.unexplored_zones = [{
        "id": "frontier_30_0",
        "pose": {"x": 3.0, "y": 0.0, "yaw": 0.0},
    }]

    search = orchestrator.decide_once()

    assert search.decision.action == ActionType.SEARCH
    assert search.submission is not None
    assert navigation.calls[-1].action == ActionType.SEARCH
    assert navigation.calls[-1].target == "frontier_30_0"
    assert orchestrator.process_results(navigation.result_queue) == 1

    observed_at = utc_now_iso()
    world.update_observation_batch(ObservationBatch(observed_at, (
        SemanticObservation(
            "fire_b_new", "fire", .9, Pose2D(3.1, 0.0), observed_at
        ),
    )))
    orchestrator.llm = StubLLM(ActionDecision(
        ActionType.NAVIGATE_TO,
        "탐색 중 발견한 화재 처리",
        "fire_b_new",
        MissionScope.FULL_EXPLORATION,
    ))

    next_fire = orchestrator.decide_once()

    assert next_fire.decision.target == "fire_b_new"
    assert next_fire.submission is not None
    assert navigation.calls[-1].target == "fire_b_new"


def test_person_threat_priority_does_not_replace_explicit_fire_only_target():
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(
            ActionType.NAVIGATE_TO,
            "명시적으로 지정된 단일 화재 처리",
            "fire_b",
            MissionScope.FIRE_ONLY,
        )
    )

    cycle = orchestrator.decide_once()

    assert world.fires["fire_a"].threatens_person is True
    assert cycle.decision.target == "fire_b"
    assert world.mission.target_fire_id == "fire_b"
    assert cycle.submission is not None
    assert navigation.calls[-1].target == "fire_b"


@pytest.mark.parametrize(
    ("fire_state", "terminal_status"),
    (
        (FireState.EXTINGUISHED, "COMPLETED"),
        (FireState.SUPPRESSION_FAILED, "COMPLETED_WITH_ESCALATION"),
    ),
)
def test_resolved_fire_mission_returns_home_before_terminal(
    fire_state,
    terminal_status,
):
    world, queue, orchestrator = make_orchestrator()
    world.bind_mission_scope(MissionScope.FIRE_ONLY, "fire_01")
    world.fires["fire_01"].state = fire_state
    orchestrator.llm = StubLLM(error=AssertionError("Qwen must not be called"))

    returning = orchestrator.decide_once()

    assert world.mission.status.value == "RUNNING"
    assert returning.decision.action == ActionType.RETURN_HOME
    assert returning.submission is not None
    assert orchestrator.llm.calls == 0

    assert orchestrator.process_results(queue) == 1
    assert world.return_home_succeeded() is True
    assert world.mission.status.value == terminal_status


def test_failed_return_home_keeps_mission_running_and_existing_retry_semantics():
    world, queue, orchestrator = make_orchestrator()
    world.bind_mission_scope(MissionScope.FIRE_ONLY, "fire_01")
    world.fires["fire_01"].state = FireState.EXTINGUISHED
    orchestrator.dispatcher.navigation.next_result = ActionResultStatus.FAILED

    first = orchestrator.decide_once()
    assert first.decision.action == ActionType.RETURN_HOME
    assert orchestrator.process_results(queue) == 1

    retry = orchestrator.decide_once()

    assert world.mission.status.value == "RUNNING"
    assert world.return_home_succeeded() is False
    assert retry.decision.action == ActionType.RETURN_HOME
    assert retry.submission is not None
    assert len(orchestrator.dispatcher.navigation.calls) == 2


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
    world, queue, orchestrator = make_orchestrator()
    orchestrator.llm = wait_llm()
    orchestrator.decide_once()
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_02", "person", .9, Pose2D(7, 0), now),
    )))
    orchestrator.decide_once()
    assert orchestrator.llm.calls == 2
    assert orchestrator.process_results(queue) == 1
    world.set_mission("m2", "새로운 임무")
    world.update_observation_batch(ObservationBatch(utc_now_iso(), tuple()))
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
    world.update_robot_pose(Pose2D(1.75, 0))
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
    world.update_robot_pose(Pose2D(1.75, 0))
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


@pytest.mark.parametrize("invalid_state", ["resolved", "invalid"])
def test_invalid_fire_does_not_continue_to_extinguish(invalid_state):
    world, queue, _, spray, llm, orchestrator = make_navigation_continuation_orchestrator()
    orchestrator.decide_once()
    world.update_robot_pose(Pose2D(1.75, 0))
    if invalid_state == "resolved":
        world.fires["fire_01"].state = world.fires["fire_01"].state.EXTINGUISHED
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
    world.update_robot_pose(Pose2D(1.75, 0))
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


def test_new_person_during_navigation_does_not_interrupt_selected_fire():
    world, queue, _, spray, llm, orchestrator = make_navigation_continuation_orchestrator()
    orchestrator.decide_once()
    world.update_robot_pose(Pose2D(1.75, 0))
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_01", "fire", .9, Pose2D(2, 0), now),
        SemanticObservation("person_01", "person", .9, Pose2D(2.05, 0), now),
    )))
    assert orchestrator.process_results(queue) == 1

    orchestrator.decide_once()

    assert len(spray.calls) == 1
    assert spray.calls[0].target == "fire_01"
    assert llm.calls == 1


@pytest.mark.parametrize("change", ["other_fire", "confidence", "observation_gap"])
def test_selected_fire_continues_without_replanning_unrelated_observations(change):
    world, queue, _, spray, llm, orchestrator = make_navigation_continuation_orchestrator()
    orchestrator.decide_once()
    world.update_robot_pose(Pose2D(1.75, 0))
    if change == "observation_gap":
        from fire_vla_core.domain import utc_now
        world.fires["fire_01"].last_seen = (utc_now() - timedelta(seconds=22)).isoformat()
    else:
        now = utc_now_iso()
        observations = [SemanticObservation("fire_01", "fire", .30, Pose2D(2, 0), now)]
        if change == "other_fire":
            observations.append(SemanticObservation("fire_b", "fire", .9, Pose2D(4, 1), now))
        world.update_observation_batch(ObservationBatch(now, tuple(observations)))
    assert orchestrator.process_results(queue) == 1
    cycle = orchestrator.decide_once()
    assert cycle.decision.action == ActionType.EXTINGUISH
    assert cycle.decision.target == "fire_01"
    assert len(spray.calls) == 1
    assert llm.calls == 1


@pytest.mark.parametrize("run", range(10))
def test_bounded_ab_execution_finishes_a_then_aligns_b_and_returns(monkeypatch, run):
    from dataclasses import replace
    from fire_vla_core import orchestrator as orch_module, validator as validator_module, world_model as world_module
    from fire_vla_core.domain import utc_now
    clock = [utc_now()]
    world, navigation, orchestrator = make_person_two_fire_orchestrator(
        ActionDecision(ActionType.NAVIGATE_TO, "A 우선",
                       "fire_a", MissionScope.FULL_EXPLORATION)
    )
    clock[0] = utc_now()
    for module in (orch_module, validator_module, world_module):
        monkeypatch.setattr(module, "utc_now", lambda: clock[0])
    world.mission.id = f"bounded_ab_{run}"
    world.config.ab_demo_mode = True
    world.people["person_a"].reported = True
    world.people["person_a"].state = PersonState.REPORTED
    queue = navigation.result_queue
    spray = orchestrator.dispatcher.spray
    first = orchestrator.decide_once()
    assert first.decision.target == "fire_a"
    # Another fire's position changes during A approach: do not replan A.
    world.fires["fire_b"].position = Pose2D(4, .01)
    world.update_robot_pose(Pose2D(1.8, 0), clock[0].isoformat())
    assert orchestrator.process_results(queue) == 1
    assert orchestrator.decide_once().decision.action == ActionType.EXTINGUISH
    assert orchestrator.llm.calls == 1
    result, = queue.drain_results()
    queue.emit(replace(result, attempts=3, timestamp=clock[0].isoformat()))
    orchestrator.process_results(queue)
    assert world.fires["fire_a"].state == FireState.PENDING_VERIFICATION
    assert orchestrator.decide_once().blocked_reason == "SUPPRESSION_PENDING_VERIFICATION"
    start = clock[0]
    for i, seconds in enumerate((.6, 1.0, 5.0)):
        clock[0] = start + timedelta(seconds=seconds)
        stamp = clock[0].isoformat()
        world.update_observation_batch(ObservationBatch(stamp, (
            SemanticObservation("fire_b", "fire", .9, Pose2D(4, .01), stamp),
            SemanticObservation("third_fire", "fire", .9, Pose2D(6, 0), stamp),
        )))
        if i < 2:
            assert world.fires["fire_a"].state == FireState.PENDING_VERIFICATION
    assert world.fires["fire_a"].state == FireState.EXTINGUISHED
    assert "third_fire" not in world.fires
    world.update_robot_pose(Pose2D(1.8, 0), clock[0].isoformat())
    second = orchestrator.decide_once()
    assert second.decision.action == ActionType.NAVIGATE_TO
    assert second.decision.target == "fire_b"
    assert [a.target for a in spray.calls] == ["fire_a"]
    world.update_robot_pose(Pose2D(3.75, .01), clock[0].isoformat())
    orchestrator.process_results(queue)
    assert orchestrator.decide_once().decision.target == "fire_b"
    assert [a.target for a in spray.calls] == ["fire_a", "fire_b"]
    result, = queue.drain_results()
    queue.emit(replace(result, status=ActionResultStatus.ABORTED, attempts=3,
                       verification_required=True, timestamp=clock[0].isoformat()))
    orchestrator.process_results(queue)
    clock[0] += timedelta(seconds=.6)
    stamp = clock[0].isoformat()
    world.update_observation_batch(ObservationBatch(stamp, (
        SemanticObservation("fire_b", "fire", .9, Pose2D(4, .01), stamp),
    )))
    world.update_robot_pose(Pose2D(3.75, .01), stamp)
    world.mark_exploration_completed()
    returning = orchestrator.decide_once()
    assert world.fires["fire_b"].state == FireState.SUPPRESSION_FAILED
    assert returning.decision.action == ActionType.RETURN_HOME
    orchestrator.process_results(queue)
    assert world.mission.status.value == "COMPLETED_WITH_ESCALATION"
    assert [a.target for a in navigation.calls] == ["fire_a", "fire_b", None]
    assert [f.spray_count for f in world.fires.values()] == [3, 3]
    assert len(spray.calls) == 2


@pytest.mark.parametrize("boundary", ["stop", "new_mission"])
def test_selected_fire_continuation_cannot_cross_stop_or_new_mission(boundary):
    world, queue, _, spray, llm, orchestrator = make_navigation_continuation_orchestrator()
    prepare_successful_navigation(world, queue, orchestrator)
    if boundary == "stop":
        world.abort_mission()
        assert orchestrator.decide_once().submission is None
    else:
        world.set_mission("next_mission", "화재 확인")
        now = utc_now_iso()
        world.update_observation_batch(ObservationBatch(now, (
            SemanticObservation("fire_01", "fire", .9, Pose2D(2, 0), now),
        )))
        llm.decision = ActionDecision(ActionType.WAIT, "새 임무 대기",
                                      mission_scope=MissionScope.FIRE_ONLY)
        orchestrator.decide_once()
        assert llm.calls == 2
    assert spray.calls == []


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


def test_fire_only_target_lock_blocks_duplicate_fire_suppression():
    world, queue, orchestrator = make_orchestrator()
    world.fires["fire_01"].position = Pose2D(0.5, 0)
    world.update_robot_pose(Pose2D(0.2, 0))
    orchestrator.llm = StubLLM(ActionDecision(
        ActionType.EXTINGUISH,
        "최초 대상 화점 진압",
        "fire_01",
        MissionScope.FIRE_ONLY,
    ))

    first = orchestrator.decide_once()
    assert first.submission is not None
    assert orchestrator.process_results(queue) == 1
    assert world.mission.target_fire_id == "fire_01"

    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_02", "fire", .9, Pose2D(.49, 0), now),
    )))
    orchestrator.llm.decision = ActionDecision(
        ActionType.EXTINGUISH,
        "새 ID 화점 진압",
        "fire_02",
        MissionScope.FIRE_ONLY,
    )
    duplicate = orchestrator.decide_once()

    assert duplicate.submission is None
    assert duplicate.blocked_reason.startswith("MISSION_TARGET_MISMATCH:")
    assert len(orchestrator.dispatcher.spray.calls) == 1


def test_new_mission_resets_fire_only_target_lock():
    world, queue, orchestrator = make_orchestrator()
    world.fires["fire_01"].position = Pose2D(0.5, 0)
    world.update_robot_pose(Pose2D(0.2, 0))
    orchestrator.llm = StubLLM(ActionDecision(
        ActionType.EXTINGUISH,
        "첫 Mission target",
        "fire_01",
        MissionScope.FIRE_ONLY,
    ))
    assert orchestrator.decide_once().submission is not None
    assert orchestrator.process_results(queue) == 1

    world.set_mission("m2", "새 화재를 진압해")
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("fire_02", "fire", .9, Pose2D(.49, 0), now),
    )))
    orchestrator.llm.decision = ActionDecision(
        ActionType.EXTINGUISH,
        "새 Mission target",
        "fire_02",
        MissionScope.FIRE_ONLY,
    )
    second = orchestrator.decide_once()

    assert second.submission is not None
    assert world.mission.target_fire_id == "fire_02"
    assert len(orchestrator.dispatcher.spray.calls) == 2
