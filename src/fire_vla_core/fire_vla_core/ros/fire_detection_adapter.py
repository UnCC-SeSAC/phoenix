"""Pure adapter for image_pipeline ``/fire/detections`` JSON envelopes."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import UTC, datetime
import math
from typing import Any, Callable


SUPPORTED_CLASSES = {"person", "fire"}
DEPTH_STATUSES = {
    "ok",
    "unknown",
    "fallback_bottom",
    "fallback_below",
    "fallback_ring",
}
HEALTH_STATES = {"ok", "stalled", "waiting_camera_info", "no_input"}


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    frame_id: str
    k: tuple[float, ...]

    @classmethod
    def from_camera_info(cls, message: Any) -> "CameraModel":
        model = cls(
            int(message.width),
            int(message.height),
            str(message.header.frame_id).strip(),
            tuple(float(value) for value in message.k),
        )
        model.validate()
        return model

    def validate(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("CameraInfo width/height는 양수여야 합니다.")
        if not self.frame_id:
            raise ValueError("CameraInfo frame_id가 필요합니다.")
        if len(self.k) != 9 or not all(math.isfinite(value) for value in self.k):
            raise ValueError("CameraInfo K는 유한한 길이 9 배열이어야 합니다.")
        if self.k[0] <= 0.0 or self.k[4] <= 0.0:
            raise ValueError("CameraInfo fx/fy는 양수여야 합니다.")


TransformPoint = Callable[
    [tuple[float, float, float], tuple[int, int], str],
    tuple[float, float, float] | None,
]


def adapt_detection_envelope(
    data: dict[str, Any],
    camera: CameraModel,
    transform_point: TransformPoint,
) -> dict[str, Any]:
    """Project an image_pipeline event into the canonical map-frame batch."""
    if not isinstance(data, dict):
        raise ValueError("/fire/detections payload는 객체여야 합니다.")
    camera.validate()
    stamp = _source_stamp(data)
    frame_size = data.get("frame_size")
    if not (
        isinstance(frame_size, list)
        and len(frame_size) == 2
        and all(_is_integer(value) for value in frame_size)
    ):
        raise ValueError("frame_size는 [width, height] 정수 배열이어야 합니다.")
    if tuple(frame_size) != (camera.width, camera.height):
        raise ValueError("frame_size와 CameraInfo 해상도가 일치해야 합니다.")
    raw_detections = data.get("detections")
    if not isinstance(raw_detections, list):
        raise ValueError("detections는 배열이어야 합니다.")

    canonical: list[dict[str, Any]] = []
    for raw in raw_detections:
        if not isinstance(raw, dict):
            raise ValueError("각 detection은 객체여야 합니다.")
        class_name = str(raw.get("class_name", "")).strip().lower()
        if class_name == "smoke":
            continue
        if class_name not in SUPPORTED_CLASSES:
            raise ValueError(f"지원하지 않는 class_name입니다: {class_name}")

        score = _finite_number(raw.get("score"), "score")
        if not 0.0 <= score <= 1.0:
            raise ValueError("score는 0과 1 사이여야 합니다.")
        pixel_x = _finite_number(raw.get("x"), "x")
        pixel_y = _finite_number(raw.get("y"), "y")
        if not 0.0 <= pixel_x < camera.width or not 0.0 <= pixel_y < camera.height:
            raise ValueError("pixel x/y가 frame_size 범위 밖입니다.")

        depth_status = str(raw.get("depth_status", ""))
        if depth_status not in DEPTH_STATUSES:
            raise ValueError(f"지원하지 않는 depth_status입니다: {depth_status}")
        # Unknown is fail-closed even if an upstream bug includes a numeric depth.
        if depth_status == "unknown":
            continue
        depth = _finite_number(raw.get("depth"), "depth")
        if depth <= 0.0:
            raise ValueError("depth는 양의 유한한 meter 값이어야 합니다.")

        optical = backproject(pixel_x, pixel_y, depth, camera.k)
        mapped = transform_point(optical, stamp, camera.frame_id)
        if mapped is None:
            continue
        map_x, map_y, map_z = (
            _finite_number(value, "transformed point") for value in mapped
        )
        canonical.append({
            "class_name": class_name,
            "confidence": score,
            "map_position": {"x": map_x, "y": map_y},
            "source_pixel": {"x": pixel_x, "y": pixel_y},
            "depth_m": depth,
            "depth_status": depth_status,
            "map_z_m": map_z,
        })

    return {
        "timestamp": _iso_timestamp(*stamp),
        "frame_id": "map",
        "frame_valid": True,
        "detector_healthy": True,
        "detections": canonical,
    }


def adapt_health_status(data: dict[str, Any]) -> dict[str, Any]:
    """Map the independent image_pipeline heartbeat to canonical health flags."""
    if not isinstance(data, dict):
        raise ValueError("/fire/detections/status payload는 객체여야 합니다.")
    stamp = _source_stamp(data)
    state = str(data.get("state", ""))
    if state not in HEALTH_STATES:
        raise ValueError(f"지원하지 않는 perception health state입니다: {state}")
    healthy = state == "ok"
    return {
        "timestamp": _iso_timestamp(*stamp),
        "frame_id": "map",
        "frame_valid": healthy,
        "detector_healthy": healthy,
        "detections": [],
        "perception_health": {
            "state": state,
            "last_frame_sec": data.get("last_frame_sec"),
            "last_frame_nanosec": data.get("last_frame_nanosec"),
            "age_sec": data.get("age_sec"),
        },
    }


def backproject(
    pixel_x: float,
    pixel_y: float,
    depth: float,
    k: tuple[float, ...],
) -> tuple[float, float, float]:
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    return (
        (pixel_x - cx) * depth / fx,
        (pixel_y - cy) * depth / fy,
        depth,
    )


def apply_transform(point: tuple[float, float, float], transform: Any) -> tuple[float, float, float]:
    """Apply a TransformStamped/Transform using ROS quaternion order x,y,z,w."""
    inner = getattr(transform, "transform", transform)
    translation = inner.translation
    rotation = inner.rotation
    qx, qy, qz, qw = (
        float(rotation.x),
        float(rotation.y),
        float(rotation.z),
        float(rotation.w),
    )
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("TF quaternion이 유효하지 않습니다.")
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    x, y, z = point
    # q * v * q^-1, expanded without adding a geometry dependency.
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)
    return (
        rx + float(translation.x),
        ry + float(translation.y),
        rz + float(translation.z),
    )


def _source_stamp(data: dict[str, Any]) -> tuple[int, int]:
    sec = data.get("stamp_sec")
    nanosec = data.get("stamp_nanosec")
    if not _is_integer(sec) or not _is_integer(nanosec):
        raise ValueError("stamp_sec/stamp_nanosec는 정수여야 합니다.")
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError("source timestamp 범위가 유효하지 않습니다.")
    return sec, nanosec


def _is_integer(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field}는 숫자여야 합니다.")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{field}는 유한한 값이어야 합니다.")
    return converted


def _iso_timestamp(seconds: int, nanoseconds: int) -> str:
    base = datetime.fromtimestamp(seconds, tz=UTC)
    return f"{base:%Y-%m-%dT%H:%M:%S}.{nanoseconds:09d}+00:00"
