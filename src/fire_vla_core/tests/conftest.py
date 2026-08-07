from __future__ import annotations

from fire_vla_core.domain import ObservationBatch, Pose2D, SemanticObservation, utc_now_iso
from fire_vla_core.world_model import WorldModel


def add_targets(world: WorldModel) -> None:
    now = utc_now_iso()
    world.update_observation_batch(ObservationBatch(now, (
        SemanticObservation("person_01", "person", 0.9, Pose2D(2.0, 0.0), now),
        SemanticObservation("fire_01", "fire", 0.9, Pose2D(0.5, 0.0), now),
    )))
