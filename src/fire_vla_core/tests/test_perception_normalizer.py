from datetime import timedelta

import pytest

from fire_vla_core.domain import (
    FireEntity,
    FireState,
    PersonEntity,
    Pose2D,
    utc_now,
)
from fire_vla_core.ros.perception_normalizer import (
    CanonicalPerceptionNormalizer,
)
from fire_vla_core.world_model import WorldModel


def detection(class_name="person", x=2.0, y=1.0, **extra):
    value = {
        "class_name": class_name,
        "confidence": 0.9,
        "map_position": {"x": x, "y": y},
    }
    value.update(extra)
    return value


def payload(*detections, timestamp=None, frame_id="map", **extra):
    value = {
        "timestamp": timestamp or utc_now().isoformat(),
        "frame_id": frame_id,
        "detections": list(detections),
    }
    value.update(extra)
    return value


def apply(normalizer, world, data):
    batch = normalizer.normalize(data)
    world.update_observation_batch(batch)
    return batch


def test_first_idless_person_gets_generated_id():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)

    batch = apply(normalizer, world, payload(detection()))

    assert batch.observations[0].entity_id == "person_0001"
    assert list(world.people) == ["person_0001"]


def test_nearby_person_in_next_batch_keeps_id_and_updates_position():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)
    apply(normalizer, world, payload(detection(x=2.0, y=1.0)))

    apply(normalizer, world, payload(detection(x=2.08, y=1.04)))

    assert list(world.people) == ["person_0001"]
    assert world.people["person_0001"].position == Pose2D(2.08, 1.04)


def test_person_outside_radius_gets_new_id():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)
    apply(normalizer, world, payload(detection(x=2.0, y=1.0)))

    apply(normalizer, world, payload(detection(x=2.6, y=1.0)))

    assert list(world.people) == ["person_0001", "person_0002"]


def test_person_and_fire_at_same_position_never_associate():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)

    apply(normalizer, world, payload(
        detection("person", 2.0, 1.0),
        detection("fire", 2.0, 1.0),
    ))

    assert list(world.people) == ["person_0001"]
    assert list(world.fires) == ["fire_0001"]


def test_nonempty_upstream_id_is_preserved():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)

    batch = apply(normalizer, world, payload(
        detection(entity_id="tracker-person-42")
    ))

    assert batch.observations[0].entity_id == "tracker-person-42"
    assert list(world.people) == ["tracker-person-42"]


def test_batch_association_is_one_to_one():
    world = WorldModel()
    now = utc_now().isoformat()
    world.people["person_0001"] = PersonEntity(
        "person_0001", Pose2D(0.0, 0.0), last_seen=now
    )
    normalizer = CanonicalPerceptionNormalizer(world)

    batch = normalizer.normalize(payload(
        detection(x=0.1, y=0.0),
        detection(x=0.2, y=0.0),
    ))

    ids = [item.entity_id for item in batch.observations]
    assert ids == ["person_0001", "person_0002"]
    assert len(set(ids)) == 2


def test_mission_reset_restarts_fallback_entity_association_ids():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)
    first = apply(normalizer, world, payload(detection()))
    assert first.observations[0].entity_id == "person_0001"

    world.set_mission("mission_02", "새 임무")
    normalizer.reset_associations()
    second = normalizer.normalize(payload(detection(x=3.0, y=2.0)))

    assert world.people == {}
    assert second.observations[0].entity_id == "person_0001"


def test_candidate_older_than_ttl_is_not_reused():
    world = WorldModel()
    old = (utc_now() - timedelta(seconds=3)).isoformat()
    world.people["person_0001"] = PersonEntity(
        "person_0001", Pose2D(2.0, 1.0), last_seen=old
    )
    normalizer = CanonicalPerceptionNormalizer(world)

    batch = normalizer.normalize(payload(detection(x=2.0, y=1.0)))

    assert batch.observations[0].entity_id == "person_0002"


def test_active_fire_keeps_id_after_association_ttl():
    world = WorldModel()
    old = (utc_now() - timedelta(seconds=8)).isoformat()
    world.fires["fire_0001"] = FireEntity(
        "fire_0001", Pose2D(2.0, 1.0), last_seen=old
    )
    normalizer = CanonicalPerceptionNormalizer(world)

    batch = normalizer.normalize(payload(detection("fire", 2.08, 1.01)))

    assert batch.observations[0].entity_id == "fire_0001"


def test_distant_fire_remains_separate_after_association_ttl():
    world = WorldModel()
    old = (utc_now() - timedelta(seconds=8)).isoformat()
    world.fires["fire_0001"] = FireEntity(
        "fire_0001", Pose2D(2.0, 1.0), last_seen=old
    )
    normalizer = CanonicalPerceptionNormalizer(world)

    batch = normalizer.normalize(payload(detection("fire", 2.6, 1.0)))

    assert batch.observations[0].entity_id == "fire_0002"


def test_resolved_fire_id_is_not_reused_after_association_ttl():
    world = WorldModel()
    old = (utc_now() - timedelta(seconds=8)).isoformat()
    world.fires["fire_0001"] = FireEntity(
        "fire_0001",
        Pose2D(2.0, 1.0),
        state=FireState.EXTINGUISHED,
        last_seen=old,
    )
    normalizer = CanonicalPerceptionNormalizer(world)

    batch = normalizer.normalize(payload(detection("fire", 2.02, 1.0)))

    assert batch.observations[0].entity_id == "fire_0002"


def test_equal_distance_tie_breaks_by_entity_id():
    world = WorldModel()
    now = utc_now().isoformat()
    world.people["person_b"] = PersonEntity(
        "person_b", Pose2D(0.1, 0.0), last_seen=now
    )
    world.people["person_a"] = PersonEntity(
        "person_a", Pose2D(-0.1, 0.0), last_seen=now
    )
    normalizer = CanonicalPerceptionNormalizer(world)

    batch = normalizer.normalize(payload(detection(x=0.0, y=0.0)))

    assert batch.observations[0].entity_id == "person_a"


def test_map_frame_is_accepted():
    normalizer = CanonicalPerceptionNormalizer(WorldModel())
    assert normalizer.normalize(payload(detection())).observations


def test_non_map_frame_is_rejected():
    normalizer = CanonicalPerceptionNormalizer(WorldModel())
    with pytest.raises(ValueError, match="frame_id"):
        normalizer.normalize(payload(detection(), frame_id="camera_link"))


@pytest.mark.parametrize("coordinate", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_coordinates_are_rejected(coordinate):
    normalizer = CanonicalPerceptionNormalizer(WorldModel())
    with pytest.raises(ValueError, match="유한한"):
        normalizer.normalize(payload(detection(x=coordinate)))


def test_malformed_or_naive_timestamp_is_rejected():
    normalizer = CanonicalPerceptionNormalizer(WorldModel())
    with pytest.raises(ValueError, match="ISO-8601"):
        normalizer.normalize(payload(detection(), timestamp="not-a-time"))
    with pytest.raises(ValueError, match="timezone"):
        normalizer.normalize(payload(detection(), timestamp="2026-08-10T02:00:00"))


@pytest.mark.parametrize("confidence", [float("nan"), -0.1, 1.1])
def test_invalid_confidence_is_rejected(confidence):
    normalizer = CanonicalPerceptionNormalizer(WorldModel())
    with pytest.raises(ValueError, match="confidence"):
        normalizer.normalize(payload(detection(confidence=confidence)))


def test_valid_below_threshold_confidence_is_ignored_by_world_model():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)

    apply(normalizer, world, payload(detection(confidence=0.4)))

    assert world.people == {}


def test_stale_batch_does_not_allocate_or_create_entity():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)
    stale = (utc_now() - timedelta(seconds=5)).isoformat()

    batch = apply(
        normalizer,
        world,
        payload(detection(), timestamp=stale),
    )

    assert batch.observations == tuple()
    assert world.people == {}
    assert world.event_log[-1].event_type == "STALE_OBSERVATION_IGNORED"


def test_two_near_frames_produce_one_person_in_snapshot():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)
    apply(normalizer, world, payload(detection(x=2.0, y=1.0)))
    apply(normalizer, world, payload(detection(x=2.05, y=1.03)))

    snapshot = world.create_snapshot()

    assert len(snapshot["people"]) == 1
    assert snapshot["people"][0]["id"] == "person_0001"
    assert snapshot["people"][0]["position"]["x"] == 2.05


def test_distant_people_produce_two_snapshot_entities():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)
    apply(normalizer, world, payload(detection(x=0.0, y=0.0)))
    apply(normalizer, world, payload(detection(x=1.0, y=0.0)))

    assert len(world.create_snapshot()["people"]) == 2


def test_person_and_fire_are_stable_in_snapshot():
    world = WorldModel()
    normalizer = CanonicalPerceptionNormalizer(world)
    apply(normalizer, world, payload(
        detection("person", 1.0, 1.0),
        detection("fire", 2.0, 2.0),
    ))
    apply(normalizer, world, payload(
        detection("person", 1.05, 1.02),
        detection("fire", 2.03, 2.04),
    ))

    snapshot = world.create_snapshot()

    assert [item["id"] for item in snapshot["people"]] == ["person_0001"]
    assert [item["id"] for item in snapshot["fires"]] == ["fire_0001"]

@pytest.mark.parametrize(
    ("field", "value"),
    [("frame_valid", "false"), ("detector_healthy", 1)],
)
def test_health_flags_require_boolean(field, value):
    normalizer = CanonicalPerceptionNormalizer(WorldModel())
    with pytest.raises(ValueError, match="boolean"):
        normalizer.normalize(payload(detection(), **{field: value}))


def test_non_string_upstream_id_is_rejected():
    normalizer = CanonicalPerceptionNormalizer(WorldModel())
    with pytest.raises(ValueError, match="entity_id"):
        normalizer.normalize(payload(detection(entity_id=42)))
