from __future__ import annotations

import json
from typing import Any

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .fire_detection_adapter import (
    CameraModel,
    adapt_detection_envelope,
    adapt_health_status,
    apply_transform,
)


class VLAPerceptionBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("vla_perception_bridge")
        self.declare_parameter("input_topic", "/fire/detections")
        self.declare_parameter("status_topic", "/fire/detections/status")
        self.declare_parameter("camera_info_topic", "/ascamera/camera_publisher/rgb0/camera_info")
        self.declare_parameter("output_topic", "/vla/perception_observation")
        self.declare_parameter("tf_lookup_timeout_sec", 2.0)
        self._camera: CameraModel | None = None
        self._tf_lookup_timeout = Duration(
            seconds=float(self.get_parameter("tf_lookup_timeout_sec").value)
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            String, self.get_parameter("output_topic").value, 10
        )
        self.create_subscription(
            String,
            self.get_parameter("input_topic").value,
            self._on_detection,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            self.get_parameter("status_topic").value,
            self._on_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._on_camera_info,
            qos_profile_sensor_data,
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        try:
            self._camera = CameraModel.from_camera_info(message)
        except ValueError as exc:
            self.get_logger().warning(f"CameraInfo rejected: {exc}")

    def _on_detection(self, message: String) -> None:
        try:
            raw = json.loads(message.data)
            if self._camera is None:
                raise ValueError("rgb0 CameraInfo를 아직 받지 못했습니다.")
            canonical = adapt_detection_envelope(
                raw, self._camera, self._transform_point
            )
        except (json.JSONDecodeError, ValueError, OverflowError) as exc:
            self.get_logger().warning(f"Perception detection dropped: {exc}")
            return
        self._publish_canonical(canonical)

    def _on_status(self, message: String) -> None:
        try:
            canonical = adapt_health_status(json.loads(message.data))
        except (json.JSONDecodeError, ValueError, OverflowError) as exc:
            self.get_logger().warning(f"Perception status dropped: {exc}")
            return
        self._publish_canonical(canonical)

    def _transform_point(self, point, stamp, source_frame):
        try:
            transform = self._tf_buffer.lookup_transform(
                "map",
                source_frame,
                Time(seconds=stamp[0], nanoseconds=stamp[1]),
                timeout=self._tf_lookup_timeout,
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"Source-time TF unavailable ({source_frame} -> map): {exc}"
            )
            return None
        return apply_transform(point, transform)

    def _publish_canonical(self, canonical: dict[str, Any]) -> None:
        output = String()
        output.data = json.dumps(canonical, allow_nan=False)
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VLAPerceptionBridgeNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
