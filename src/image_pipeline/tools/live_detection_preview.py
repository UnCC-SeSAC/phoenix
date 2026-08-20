#!/usr/bin/env python3
"""Show the exact YOLO input frame with matching detection overlays."""

from __future__ import annotations

import argparse
import os
import sys
from collections import OrderedDict

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from image_pipeline.detection_msgs import bbox_center, hypothesis  # noqa: E402


def stamp_key(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class DetectionPreview(Node):
    def __init__(self, image_topic: str, detections_topic: str) -> None:
        super().__init__("live_detection_preview")
        self._bridge = CvBridge()
        self._frames: OrderedDict[int, object] = OrderedDict()
        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            Detection2DArray, detections_topic, self._on_detections,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"preview exact inference frames: {image_topic} + {detections_topic}"
        )

    def _on_image(self, msg: Image) -> None:
        self._frames[stamp_key(msg.header)] = self._bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8"
        )
        while len(self._frames) > 30:
            self._frames.popitem(last=False)

    def _on_detections(self, msg: Detection2DArray) -> None:
        frame = self._frames.pop(stamp_key(msg.header), None)
        if frame is None:
            return
        for detection in msg.detections:
            if not detection.results:
                continue
            class_id, score = hypothesis(detection.results[0])
            center_x, center_y = bbox_center(detection.bbox)
            width, height = float(detection.bbox.size_x), float(detection.bbox.size_y)
            x1, y1 = int(center_x - width / 2), int(center_y - height / 2)
            x2, y2 = int(center_x + width / 2), int(center_y + height / 2)
            label = f"{class_id} {score:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame, label, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 255, 0), 2, cv2.LINE_AA,
            )
        cv2.putText(
            frame, f"detections: {len(msg.detections)}", (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2, cv2.LINE_AA,
        )
        cv2.imshow("Phoenix HEF live detection", frame)
        cv2.waitKey(1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/image_enhanced")
    parser.add_argument("--detections-topic", default="/yolo_result")
    args = parser.parse_args()
    rclpy.init()
    node = DetectionPreview(args.image_topic, args.detections_topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
