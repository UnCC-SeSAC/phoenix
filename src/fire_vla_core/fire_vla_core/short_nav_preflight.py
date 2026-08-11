"""Hardware-free deterministic preflight for the first VLA short navigation."""

from __future__ import annotations

import json

from .adapters.mock_adapters import (
    MockNavigationAdapter,
    MockReportAdapter,
    MockResultQueue,
    MockSprayAdapter,
    MockWaitAdapter,
)
from .dispatcher import ActionDispatcher
from .domain import ActionDecision, ActionType, Pose2D, utc_now_iso
from .orchestrator import VLAOrchestrator
from .resolver import TargetResolver
from .ros.perception_normalizer import CanonicalPerceptionNormalizer
from .validator import ActionValidator
from .world_model import WorldModel


class ShortNavDecisionStub:
    """Gate-only decision source; it does not replace general MockVLABrain."""

    def decide(self, mission: str, world_model: dict) -> ActionDecision:
        people = world_model.get("people") or []
        if not any(person.get("id") == "person_0001" for person in people):
            raise ValueError("short-nav preflight requires person_0001")
        return ActionDecision(
            ActionType.NAVIGATE_TO,
            "첫 short-nav transport 검증 대상에게 이동한다",
            "person_0001",
        )


def run_short_nav_preflight() -> dict:
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)

    # Model the live boundary as a stream: every sample carries a fresh source
    # timestamp and refreshes WorldModel.pose_updated_at.
    for _ in range(3):
        world.update_robot_pose(Pose2D(0.0, 0.0, 0.0), utc_now_iso())

    observed_at = utc_now_iso()
    world.update_observation_batch(normalizer.normalize({
        "timestamp": observed_at,
        "frame_id": "map",
        "frame_valid": True,
        "detector_healthy": True,
        "detections": [{
            "class_name": "person",
            "confidence": 0.99,
            "map_position": {"x": 0.5, "y": 0.0, "yaw": 0.0},
        }],
    }))
    world.set_mission("vla07_short_nav_preflight", "인명을 우선 확인해.")

    results = MockResultQueue()
    navigation = MockNavigationAdapter(results)
    dispatcher = ActionDispatcher(
        navigation,
        MockSprayAdapter(results),
        MockReportAdapter(results),
        MockWaitAdapter(results),
    )
    cycle = VLAOrchestrator(
        world,
        ShortNavDecisionStub(),
        TargetResolver(),
        ActionValidator(),
        dispatcher,
    ).decide_once()

    action = cycle.validation.action if cycle.validation else None
    return {
        "robot_pose_fresh": bool(cycle.validation and cycle.validation.approved),
        "person_ids": sorted(world.people),
        "decision": cycle.decision.action.value if cycle.decision else None,
        "decision_target": cycle.decision.target if cycle.decision else None,
        "resolved_target_pose": (
            {
                "x": action.target_pose.x,
                "y": action.target_pose.y,
                "yaw": action.target_pose.yaw,
                "frame_id": "map",
            }
            if action and action.target_pose
            else None
        ),
        "validator_approved": bool(cycle.validation and cycle.validation.approved),
        "submission": cycle.submission.status.value if cycle.submission else None,
        "navigation_adapter": type(navigation).__name__,
        "mock_navigation_calls": len(navigation.calls),
        "actual_nav2_goals": 0,
        "cmd_vel_messages": 0,
    }


def main() -> None:
    print(json.dumps(run_short_nav_preflight(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
