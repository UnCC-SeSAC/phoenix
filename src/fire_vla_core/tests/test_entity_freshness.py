from datetime import timedelta
from math import nan

import pytest

from fire_vla_core.adapters.mock_adapters import (
    MockNavigationAdapter,
    MockReportAdapter,
    MockResultQueue,
    MockSprayAdapter,
    MockWaitAdapter,
)
from fire_vla_core.dispatcher import ActionDispatcher
from fire_vla_core.domain import (
    ActionDecision,
    ActionType,
    FireState,
    ObservationBatch,
    Pose2D,
    SemanticObservation,
    utc_now,
    utc_now_iso,
)
from fire_vla_core.llm import build_compact_world_model, extract_valid_targets
from fire_vla_core.orchestrator import VLAOrchestrator
from fire_vla_core.resolver import TargetResolver
from fire_vla_core.validator import ActionValidator
from fire_vla_core.world_model import WorldModel


class FixedLLM:
    def __init__(self, decision):
        self.decision = decision
        self.calls = 0

    def decide(self, mission, world_model):
        self.calls += 1
        return self.decision


def make_world(*, person=True, fire=True, fire_x=0.5):
    world = WorldModel()
    world.update_robot_pose(Pose2D(0.0, 0.0))
    world.set_mission("freshness", "유효한 대상에만 대응해")
    now = utc_now_iso()
    observations = []
    if person:
        observations.append(SemanticObservation(
            "person_01", "person", 0.9, Pose2D(1.0, 0.0), now,
        ))
    if fire:
        observations.append(SemanticObservation(
            "fire_01", "fire", 0.9, Pose2D(fire_x, 0.0), now,
        ))
    world.update_observation_batch(ObservationBatch(now, tuple(observations)))
    return world


def make_orchestrator(world, decision):
    results = MockResultQueue()
    navigation = MockNavigationAdapter(results)
    spray = MockSprayAdapter(results)
    report = MockReportAdapter(results)
    waiter = MockWaitAdapter(results)
    dispatcher = ActionDispatcher(navigation, spray, report, waiter)
    orchestrator = VLAOrchestrator(
        world,
        FixedLLM(decision),
        TargetResolver(),
        ActionValidator(),
        dispatcher,
    )
    return orchestrator, results, navigation, spray, report, waiter


def decision_view(world):
    snapshot = world.create_snapshot()
    compact = build_compact_world_model(snapshot)
    return snapshot, compact, extract_valid_targets(compact)


def make_stale(entity):
    entity.last_seen = (utc_now() - timedelta(seconds=5)).isoformat()


def test_fresh_unreported_person_is_eligible_and_reportable():
    world = make_world(fire=False)
    snapshot, compact, targets = decision_view(world)
    assert snapshot["people"][0]["decision_eligible"] is True
    assert [item["id"] for item in compact["people"]] == ["person_01"]
    assert targets == ["person_01"]

    orchestrator, _, _, _, report, _ = make_orchestrator(
        world,
        ActionDecision(ActionType.REPORT_PERSON, "유효 위치 보고", "person_01"),
    )
    cycle = orchestrator.decide_once()
    assert cycle.validation and cycle.validation.approved
    assert len(report.calls) == 1


@pytest.mark.parametrize("invalid_kind", ["stale", "invalid_position"])
def test_invalid_person_is_history_only_and_cannot_be_reported(invalid_kind):
    world = make_world(fire=False)
    person = world.people["person_01"]
    if invalid_kind == "stale":
        make_stale(person)
    else:
        person.position = Pose2D(nan, 0.0)

    snapshot, compact, targets = decision_view(world)
    assert "person_01" in world.people
    assert snapshot["people"][0]["decision_eligible"] is False
    assert compact["people"] == []
    assert "person_01" not in targets

    orchestrator, _, _, _, report, _ = make_orchestrator(
        world,
        ActionDecision(ActionType.REPORT_PERSON, "보고 시도", "person_01"),
    )
    cycle = orchestrator.decide_once()
    assert cycle.validation and not cycle.validation.approved
    assert cycle.submission is None
    assert report.calls == []


@pytest.mark.parametrize(
    "fire_x,action_type",
    [(0.5, ActionType.EXTINGUISH), (2.0, ActionType.NAVIGATE_TO)],
)
def test_fresh_fire_is_eligible_and_uses_existing_validator(fire_x, action_type):
    world = make_world(person=False, fire_x=fire_x)
    _, compact, targets = decision_view(world)
    assert [item["id"] for item in compact["fires"]] == ["fire_01"]
    assert targets == ["fire_01"]

    orchestrator, _, navigation, spray, _, _ = make_orchestrator(
        world,
        ActionDecision(action_type, "유효 화점 대응", "fire_01"),
    )
    cycle = orchestrator.decide_once()
    assert cycle.validation and cycle.validation.approved
    assert len(spray.calls if action_type == ActionType.EXTINGUISH else navigation.calls) == 1


@pytest.mark.parametrize(
    "fire_x,action_type",
    [(0.5, ActionType.EXTINGUISH), (2.0, ActionType.NAVIGATE_TO)],
)
def test_stale_fire_is_history_only_and_never_physically_dispatched(
    fire_x,
    action_type,
):
    world = make_world(person=False, fire_x=fire_x)
    make_stale(world.fires["fire_01"])
    snapshot, compact, targets = decision_view(world)
    assert "fire_01" in world.fires
    assert snapshot["fires"][0]["decision_eligible"] is False
    assert compact["fires"] == []
    assert "fire_01" not in targets

    orchestrator, _, navigation, spray, _, _ = make_orchestrator(
        world,
        ActionDecision(action_type, "stale 화점 시도", "fire_01"),
    )
    cycle = orchestrator.decide_once()
    assert cycle.validation and not cycle.validation.approved
    assert cycle.submission is None
    assert navigation.calls == []
    assert spray.calls == []


@pytest.mark.parametrize("stale_kind", ["fire", "person"])
def test_mixed_freshness_exposes_only_the_fresh_entity(stale_kind):
    world = make_world()
    make_stale(
        world.fires["fire_01"]
        if stale_kind == "fire"
        else world.people["person_01"]
    )
    _, compact, targets = decision_view(world)
    expected = "person_01" if stale_kind == "fire" else "fire_01"
    assert targets == [expected]
    assert [item["id"] for item in compact["people"] + compact["fires"]] == [
        expected
    ]


def test_all_stale_keeps_history_but_blocks_fabricated_physical_target():
    world = make_world()
    make_stale(world.people["person_01"])
    make_stale(world.fires["fire_01"])
    snapshot, compact, targets = decision_view(world)
    assert len(snapshot["people"]) == 1
    assert len(snapshot["fires"]) == 1
    assert compact["people"] == []
    assert compact["fires"] == []
    assert targets == []

    orchestrator, _, navigation, spray, _, _ = make_orchestrator(
        world,
        ActionDecision(ActionType.NAVIGATE_TO, "없는 대상", "fire_01"),
    )
    cycle = orchestrator.decide_once()
    assert cycle.submission is None
    assert navigation.calls == []
    assert spray.calls == []


@pytest.mark.parametrize(
    "action_type,target",
    [
        (ActionType.NAVIGATE_TO, "fire_01"),
        (ActionType.EXTINGUISH, "fire_01"),
    ],
)
def test_entity_expiry_does_not_cancel_an_in_progress_action(action_type, target):
    world = make_world(person=False, fire_x=0.5 if action_type == ActionType.EXTINGUISH else 2.0)
    orchestrator, _, navigation, spray, _, waiter = make_orchestrator(
        world,
        ActionDecision(action_type, "시작", target),
    )
    first = orchestrator.decide_once()
    assert first.submission is not None
    make_stale(world.fires[target])

    waiting = orchestrator.decide_once()
    assert waiting.decision and waiting.decision.action == ActionType.WAIT
    assert world.current_action is not None
    assert len(waiter.calls) == 1
    assert len(navigation.calls) + len(spray.calls) == 1


def test_reported_person_expiry_preserves_history_without_repeat_report():
    world = make_world(fire=False)
    world.people["person_01"].reported = True
    make_stale(world.people["person_01"])
    snapshot, compact, targets = decision_view(world)
    assert snapshot["people"][0]["reported"] is True
    assert compact["people"] == []
    assert targets == []


def test_pending_verification_fire_expiry_remains_history_only():
    world = make_world(person=False)
    fire = world.fires["fire_01"]
    fire.state = FireState.PENDING_VERIFICATION
    fire.verification_started_at = utc_now_iso()
    make_stale(fire)
    snapshot, compact, targets = decision_view(world)
    assert snapshot["fires"][0]["state"] == "PENDING_VERIFICATION"
    assert compact["fires"] == []
    assert targets == []


@pytest.mark.parametrize("bad_timestamp", ["future", "", "not-a-time"])
def test_future_missing_or_malformed_entity_timestamp_is_fail_safe(
    bad_timestamp,
):
    world = make_world(fire=False)
    if bad_timestamp == "future":
        value = (utc_now() + timedelta(seconds=5)).isoformat()
    else:
        value = bad_timestamp
    world.people["person_01"].last_seen = value
    _, compact, targets = decision_view(world)
    assert compact["people"] == []
    assert targets == []
    assert not world.person_is_decision_eligible("person_01")


def test_future_observation_is_not_stored():
    world = make_world(person=False, fire=False)
    future = (utc_now() + timedelta(seconds=5)).isoformat()
    world.update_observation_batch(ObservationBatch(future, (
        SemanticObservation(
            "person_future", "person", 0.9, Pose2D(1.0, 0.0), future,
        ),
    )))
    assert "person_future" not in world.people
    assert any(
        event.event_type == "FUTURE_OBSERVATION_IGNORED"
        for event in world.event_log
    )
