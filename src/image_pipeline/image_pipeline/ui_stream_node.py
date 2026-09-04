#!/usr/bin/env python3
from __future__ import annotations

import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray

from image_pipeline.ui_stream import normalize_boxes, stream_size, throttle


def _stamp_key(header) -> tuple[int, int]:
    return (int(header.stamp.sec), int(header.stamp.nanosec))


def _stamp_sec(key: tuple[int, int]) -> float:
    return key[0] + key[1] * 1e-9


class UiStreamNode(Node):
    def __init__(self):
        super().__init__("ui_stream_node")
        self._declare_params()
        p = self.get_parameter
        self.class_names = [
            str(name) for name in (p("class_names").value or []) if str(name).strip()
        ]
        self.max_width = int(p("stream_max_width").value)
        self.quality = int(p("jpeg_quality").value)
        self.fps = float(p("stream_fps").value)
        self.slop = float(p("slop_sec").value)
        self.publish_empty = bool(p("publish_empty").value)
        self.bridge = CvBridge()
        self._detections = {}
        self._detection_cache_size = int(p("detection_cache").value)
        self._last_emit = None
        self._sequence = 0
        self._image_topic = str(p("input_topic").value)
        self._detection_topic = str(p("detections_topic").value)
        self._image_subscription = None
        self._detection_subscription = None
        self.frame_publisher = self.create_publisher(
            CompressedImage, str(p("stream_topic").value), qos_profile_sensor_data
        )
        self.overlay_publisher = self.create_publisher(
            String, str(p("overlay_topic").value), qos_profile_sensor_data
        )
        self.enabled = bool(p("start_enabled").value)
        if self.enabled:
            self._subscribe_vision()
        self.create_subscription(
            Bool, str(p("enabled_topic").value), self._on_enabled, 10
        )

    def _declare_params(self):
        self.declare_parameter("input_topic", "/image_enhanced")
        self.declare_parameter("detections_topic", "/yolo_result")
        self.declare_parameter("stream_topic", "/ui/camera/compressed")
        self.declare_parameter("overlay_topic", "/ui/camera/overlay")
        self.declare_parameter("enabled_topic", "/ui/camera/enabled")
        self.declare_parameter("start_enabled", False)
        self.declare_parameter("class_names", [""])
        self.declare_parameter("stream_fps", 8.0)
        self.declare_parameter("stream_max_width", 640)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("slop_sec", 0.1)
        self.declare_parameter("publish_empty", True)
        self.declare_parameter("detection_cache", 30)

    def _subscribe_vision(self):
        self._detection_subscription = self.create_subscription(
            Detection2DArray, self._detection_topic, self.on_detections,
            qos_profile_sensor_data,
        )
        self._image_subscription = self.create_subscription(
            Image, self._image_topic, self.on_image, qos_profile_sensor_data
        )

    def _on_enabled(self, message: Bool) -> None:
        enabled = bool(message.data)
        if enabled == self.enabled:
            return
        self.enabled = enabled
        if enabled:
            self._subscribe_vision()
            return
        self.destroy_subscription(self._image_subscription)
        self.destroy_subscription(self._detection_subscription)
        self._image_subscription = None
        self._detection_subscription = None
        self._detections.clear()

    def on_detections(self, message: Detection2DArray):
        self._detections[_stamp_key(message.header)] = list(message.detections)
        while len(self._detections) > self._detection_cache_size:
            self._detections.pop(next(iter(self._detections)))

    def _match(self, key):
        exact = self._detections.get(key)
        if exact is not None:
            return exact
        if not self._detections:
            return None
        target = _stamp_sec(key)
        nearest = min(self._detections, key=lambda stamp: abs(_stamp_sec(stamp) - target))
        return (
            self._detections[nearest]
            if abs(_stamp_sec(nearest) - target) <= self.slop
            else None
        )

    def on_image(self, message: Image):
        now = time.monotonic()
        if not throttle(self._last_emit, now, self.fps):
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            source_height, source_width = image.shape[:2]
            detections = self._match(_stamp_key(message.header))
            if detections is None:
                if not self.publish_empty:
                    return
                detections = []
            boxes = normalize_boxes(
                detections, source_width, source_height, self.class_names
            )
            output_width, output_height = stream_size(
                source_width, source_height, self.max_width
            )
            if (output_width, output_height) != (source_width, source_height):
                image = cv2.resize(
                    image, (output_width, output_height), interpolation=cv2.INTER_AREA
                )
            encoded, buffer = cv2.imencode(
                ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            )
            if not encoded:
                return
        except Exception as exc:  # malformed input must not terminate the node
            self.get_logger().error(f"UI stream frame dropped: {exc}")
            return
        self._sequence += 1
        frame = CompressedImage()
        frame.header = message.header
        frame.format = "jpeg"
        frame.data = buffer.tobytes()
        self.frame_publisher.publish(frame)
        overlay = String()
        overlay.data = json.dumps({
            "seq": self._sequence,
            "stamp_sec": int(message.header.stamp.sec),
            "stamp_nanosec": int(message.header.stamp.nanosec),
            "width": output_width,
            "height": output_height,
            "boxes": boxes,
        }, ensure_ascii=False)
        self.overlay_publisher.publish(overlay)
        self._last_emit = now


def main(args=None):
    rclpy.init(args=args)
    node = UiStreamNode()
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
