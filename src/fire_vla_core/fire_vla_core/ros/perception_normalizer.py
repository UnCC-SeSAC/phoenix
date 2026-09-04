from __future__ import annotations
import re

from datetime import datetime
from itertools import count
from math import isfinite
from typing import Any

from ..domain import (
    ObservationBatch,
    FireState,
    Pose2D,
    SemanticObservation,
    utc_now,
)
from ..world_model import WorldModel


DEFAULT_ASSOCIATION_RADIUS_M = 0.5
DEFAULT_ASSOCIATION_TTL_SEC = 2.0
_SUPPORTED_CLASSES = {"person", "fire"}


class CanonicalPerceptionNormalizer:
    """Validate canonical map detections and assign fallback semantic IDs."""

    def __init__(
        self,
        world: WorldModel,
        *,
        association_radius_m: float = DEFAULT_ASSOCIATION_RADIUS_M,
        association_ttl_sec: float = DEFAULT_ASSOCIATION_TTL_SEC,
    ) -> None:
        if association_radius_m <= 0 or association_ttl_sec <= 0:
            raise ValueError("association radius와 TTL은 양수여야 합니다.")
        self._world = world
        self._radius_m = association_radius_m
        self._ttl_sec = association_ttl_sec
        self._id_sequences = {
            "person": count(1),
            "fire": count(1),
        }

    def reset_associations(self) -> None:
        self._id_sequences = {
            "person": count(1),
            "fire": count(1),
        }

    def normalize(self, data: dict[str, Any]) -> ObservationBatch:
        if not isinstance(data, dict):
            raise ValueError("canonical perception payload는 객체여야 합니다.")
        if data.get("frame_id") != "map":
            raise ValueError('canonical perception frame_id는 "map"이어야 합니다.')

        observed_at = str(data["timestamp"])
        observed_time = self._parse_timestamp(observed_at)
        observed_at = observed_time.isoformat()
        frame_valid = data.get("frame_valid", True)
        detector_healthy = data.get("detector_healthy", True)
        if not isinstance(frame_valid, bool) or not isinstance(detector_healthy, bool):
            raise ValueError("frame_valid와 detector_healthy는 boolean이어야 합니다.")
        raw_detections = data.get("detections", [])
        if not isinstance(raw_detections, list):
            raise ValueError("detections는 배열이어야 합니다.")

        if (
            (utc_now() - observed_time).total_seconds()
            > self._world.config.observation_max_age_sec
            or not frame_valid
            or not detector_healthy
        ):
            return ObservationBatch(
                observed_at,
                tuple(),
                frame_valid,
                detector_healthy,
            )

        used_ids: set[str] = set()
        observations: list[SemanticObservation] = []
        for raw in raw_detections:
            if not isinstance(raw, dict):
                raise ValueError("각 detection은 객체여야 합니다.")
            class_name = str(raw["class_name"]).strip().lower()
            if class_name not in _SUPPORTED_CLASSES:
                raise ValueError(f"지원하지 않는 class_name입니다: {class_name}")
            confidence = float(raw["confidence"])
            if not isfinite(confidence) or not 0.0 <= confidence <= 1.0:
                raise ValueError("confidence는 0과 1 사이의 유한한 값이어야 합니다.")

            position_data = raw["map_position"]
            if not isinstance(position_data, dict):
                raise ValueError("map_position은 객체여야 합니다.")
            position = Pose2D(
                float(position_data["x"]),
                float(position_data["y"]),
                float(position_data.get("yaw", 0.0)),
            )
            if not all(isfinite(value) for value in (
                position.x,
                position.y,
                position.yaw,
            )):
                raise ValueError("map_position에는 유한한 x, y, yaw가 필요합니다.")

            upstream_id = raw.get("entity_id")
            if upstream_id is not None and not isinstance(upstream_id, str):
                raise ValueError("entity_id는 문자열이어야 합니다.")
            entity_id = upstream_id.strip() if upstream_id is not None else ""

            if class_name == "fire" and any(
                item.class_name == "fire"
                and position.distance_to(item.position) <= self._radius_m
                for item in observations
            ):
                continue

            entities = (
                self._world.people
                if class_name == "person"
                else self._world.fires
            )
            if not entity_id or (
                class_name == "fire" and entity_id not in entities
            ):
                entity_id = self._associate(
                    class_name,
                    position,
                    observed_time,
                    used_ids,
                    fallback_id=entity_id or None,
                )
            if entity_id in used_ids:
                raise ValueError(
                    f"한 batch에서 entity_id를 중복 사용할 수 없습니다: {entity_id}"
                )
            used_ids.add(entity_id)
            observations.append(SemanticObservation(
                entity_id=entity_id,
                class_name=class_name,
                confidence=confidence,
                position=position,
                observed_at=observed_at,
                size=raw.get("size"),
                blocks_route_to=raw.get("blocks_route_to"),
            ))

        return ObservationBatch(
            observed_at,
            tuple(observations),
            frame_valid,
            detector_healthy,
        )

    def _associate(
        self,
        class_name: str,
        position: Pose2D,
        observed_time: datetime,
        used_ids: set[str],
        fallback_id: str | None = None,
    ) -> str:
        entities = (
            self._world.people
            if class_name == "person"
            else self._world.fires
        )
        candidates: list[tuple[float, str]] = []
        for entity_id, entity in entities.items():
            if entity_id in used_ids:
                continue
            last_seen = self._parse_timestamp(entity.last_seen)
            age = max(0.0, (observed_time - last_seen).total_seconds())
            person_is_trackable = (
                class_name == "person" and entity.reported
            )
            fire_is_trackable = (
                class_name == "fire"
                and entity.state in {
                    FireState.ACTIVE,
                    FireState.PENDING_VERIFICATION,
                    FireState.EXTINGUISHED,
                    FireState.INACCESSIBLE,
                }
            )
            if (
                age > self._ttl_sec
                and not person_is_trackable
                and not fire_is_trackable
            ):
                continue
            distance = position.distance_to(entity.position)
            if distance <= self._radius_m:
                candidates.append((distance, entity_id))
        if candidates:
            return min(candidates, key=lambda item: (item[0], item[1]))[1]
        if fallback_id:
            return fallback_id
        return self._new_id(class_name, entities, used_ids)

    def _new_id(
        self,
        class_name: str,
        entities: dict[str, Any],
        used_ids: set[str],
    ) -> str:
        while True:
            entity_id = f"{class_name}_{next(self._id_sequences[class_name]):04d}"
            if entity_id not in entities and entity_id not in used_ids:
                return entity_id

    @staticmethod
    def _parse_timestamp(value: str) -> datetime:
        match = re.fullmatch(
            r"(.+\.\d{6})\d+([+-]\d{2}:\d{2})", value
        )
        parseable = f"{match.group(1)}{match.group(2)}" if match else value
        try:
            parsed = datetime.fromisoformat(parseable)
        except (TypeError, ValueError) as exc:
            raise ValueError("timestamp는 유효한 ISO-8601이어야 합니다.") from exc
        if parsed.tzinfo is None or parsed.utcoffset() is None:
            raise ValueError("timestamp에는 timezone 정보가 필요합니다.")
        return parsed
