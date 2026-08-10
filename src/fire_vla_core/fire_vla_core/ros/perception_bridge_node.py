from __future__ import annotations

import json
import math
from datetime import UTC, datetime
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SUPPORTED_CLASSES = {"person", "fire"}


def to_canonical_observation(data: dict[str, Any]) -> dict[str, Any] | None:
    """Map one live map-frame detection to the canonical VLA boundary."""
    if not isinstance(data, dict):
        raise ValueError("vision detection payload는 객체여야 합니다.")

    class_name = data.get("class")
    if class_name == "smoke":
        return None
    if class_name not in SUPPORTED_CLASSES:
        raise ValueError("class는 person 또는 fire여야 합니다.")
    if data.get("frame_id") != "map":
        raise ValueError('vision detection frame_id는 "map"이어야 합니다.')

    confidence = _finite_number(data.get("confidence"), "confidence")
    if not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence는 0과 1 사이여야 합니다.")
    x = _finite_number(data.get("x"), "x")
    y = _finite_number(data.get("y"), "y")

    stamp_sec = _integer(data.get("stamp_sec"), "stamp_sec")
    stamp_nanosec = _integer(data.get("stamp_nanosec"), "stamp_nanosec")
    if stamp_sec < 0 or not 0 <= stamp_nanosec < 1_000_000_000:
        raise ValueError("timestamp 범위가 유효하지 않습니다.")
    timestamp = _iso_timestamp(stamp_sec, stamp_nanosec)

    return {
        "timestamp": timestamp,
        "frame_id": "map",
        "frame_valid": True,
        "detector_healthy": True,
        "detections": [{
            "class_name": class_name,
            "confidence": confidence,
            "map_position": {"x": x, "y": y},
        }],
    }


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field}는 숫자여야 합니다.")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{field}는 유한한 값이어야 합니다.")
    return converted


def _integer(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field}는 정수여야 합니다.")
    return value


def _iso_timestamp(seconds: int, nanoseconds: int) -> str:
    base = datetime.fromtimestamp(seconds, tz=UTC)
    return f"{base:%Y-%m-%dT%H:%M:%S}.{nanoseconds:09d}+00:00"


class VLAPerceptionBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("vla_perception_bridge")
        self.declare_parameter("input_topic", "/vision/detections")
        self.declare_parameter("output_topic", "/vla/perception_observation")
        self._publisher = self.create_publisher(
            String, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            String,
            self.get_parameter("input_topic").value,
            self._on_detection,
            10,
        )

    def _on_detection(self, message: String) -> None:
        try:
            raw = json.loads(message.data)
            canonical = to_canonical_observation(raw)
        except (json.JSONDecodeError, ValueError, OverflowError) as exc:
            self.get_logger().warning(f"Perception detection dropped: {exc}")
            return
        if canonical is None:
            return
        output = String()
        output.data = json.dumps(canonical, allow_nan=False)
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VLAPerceptionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
