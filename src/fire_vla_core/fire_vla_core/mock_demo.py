from __future__ import annotations

from datetime import timedelta

from .adapters.mock_adapters import MockNavigationAdapter, MockReportAdapter, MockResultQueue, MockSprayAdapter, MockWaitAdapter
from .dispatcher import ActionDispatcher
from .domain import ObservationBatch, Pose2D, SemanticObservation, utc_now
from .llm import MockVLABrain
from .orchestrator import VLAOrchestrator
from .resolver import TargetResolver
from .validator import ActionValidator
from .world_model import WorldModel, WorldModelConfig


def main() -> None:
    world = WorldModel(WorldModelConfig(verification_required_observations=2, verification_delay_sec=0.0))
    world.update_robot_pose(Pose2D(0.0, 0.0, 0.0))
    world.set_mission("mission_001", "인명을 우선 확인하되, 출구를 막는 소형 화점은 안전하게 진압할 수 있다면 먼저 제거해.")
    now = utc_now()
    world.update_observation_batch(ObservationBatch(now.isoformat(), (
        SemanticObservation("person_01", "person", 0.95, Pose2D(5.0, 2.0), now.isoformat()),
        SemanticObservation("fire_01", "fire", 0.92, Pose2D(2.0, 1.0), now.isoformat(), "SMALL", "person_01"),
    )))
    world.mark_exploration_completed()

    results = MockResultQueue()
    dispatcher = ActionDispatcher(
        MockNavigationAdapter(results),
        MockSprayAdapter(results),
        MockReportAdapter(results),
        MockWaitAdapter(results),
    )
    orchestrator = VLAOrchestrator(world, MockVLABrain(), TargetResolver(), ActionValidator(), dispatcher)

    for step in range(12):
        cycle = orchestrator.decide_once()
        print(f"[{step}] decision={cycle.decision} submission={cycle.submission}")
        orchestrator.process_results(results)

        last = world.last_action
        if last and last.action.value == "NAVIGATE_TO" and last.target:
            target = world.fires.get(last.target) or world.people.get(last.target)
            if target:
                world.update_robot_pose(target.position)
        if world.fires.get("fire_01") and world.fires["fire_01"].state.value == "PENDING_VERIFICATION":
            t1 = (now + timedelta(seconds=1)).isoformat()
            t2 = (now + timedelta(seconds=2)).isoformat()
            world.update_observation_batch(ObservationBatch(t1, tuple()))
            world.update_observation_batch(ObservationBatch(t2, tuple()))
        if world.mission and world.mission.status.value == "COMPLETED":
            break

    print(world.create_snapshot())


if __name__ == "__main__":
    main()
