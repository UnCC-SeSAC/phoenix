#!/usr/bin/env python3
"""
가짜 카메라 노드 — 로봇 수령 전에 태스크① 노드를 돌려보기 위한 도구.

이미지 폴더 / 동영상 파일 / USB 웹캠을 `/camera/color/image_raw`로 발행합니다.
**실제 카메라와 똑같이 BEST_EFFORT(qos_profile_sensor_data)로 발행**하므로,
QoS 함정도 그대로 재현됩니다(일부러 그렇게 했습니다 — 노드에서 정수 10을 쓰면
콜백이 안 불리는 걸 여기서 미리 겪어볼 수 있습니다).

  ros2 run image_pipeline fake_camera_node --ros-args \
      -p source:=/path/to/smoke.mp4 -p fps:=20.0
"""

from __future__ import annotations

import glob
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class FakeCamera(Node):
    def __init__(self):
        super().__init__("fake_camera_node")
        self.declare_parameter("source", "0")  # 숫자=웹캠 인덱스, 경로=파일/폴더
        self.declare_parameter("topic", "/camera/color/image_raw")
        self.declare_parameter("info_topic", "/camera/color/camera_info")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("fps", 20.0)
        self.declare_parameter("loop", True)

        src = self.get_parameter("source").value
        self.frame_id = self.get_parameter("frame_id").value
        self.loop = bool(self.get_parameter("loop").value)
        fps = float(self.get_parameter("fps").value)

        self.bridge = CvBridge()
        self.pub = self.create_publisher(
            Image, self.get_parameter("topic").value, qos_profile_sensor_data
        )
        self.info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("info_topic").value, qos_profile_sensor_data
        )

        self.images: list[str] = []
        self.cap = None
        self.idx = 0

        if os.path.isdir(src):
            for ext in ("jpg", "jpeg", "png", "bmp"):
                self.images += glob.glob(os.path.join(src, f"*.{ext}"))
            self.images.sort()
            self.get_logger().info(f"이미지 {len(self.images)}장 발행")
        else:
            self.cap = cv2.VideoCapture(int(src) if src.isdigit() else src)
            if not self.cap.isOpened():
                raise RuntimeError(f"소스를 열 수 없음: {src}")
            self.get_logger().info(f"동영상/웹캠 발행: {src}")

        self.timer = self.create_timer(1.0 / max(fps, 1e-3), self.tick)

    def next_frame(self):
        if self.images:
            if self.idx >= len(self.images):
                if not self.loop:
                    return None
                self.idx = 0
            img = cv2.imread(self.images[self.idx])
            self.idx += 1
            return img
        ok, img = self.cap.read()
        if not ok:
            if not self.loop:
                return None
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, img = self.cap.read()
        return img if ok else None

    def tick(self):
        img = self.next_frame()
        if img is None:
            self.get_logger().info("소스 끝. 종료합니다.")
            self.timer.cancel()
            return

        stamp = self.get_clock().now().to_msg()
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

        h, w = img.shape[:2]
        # 그럴듯한 가짜 K (fx≈fy≈w, cx=w/2, cy=h/2). 실물 값은 로봇 수령 후 교체.
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = w, h
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [float(w), 0.0, w / 2.0, 0.0, float(w), h / 2.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [float(w), 0.0, w / 2.0, 0.0,
                  0.0, float(w), h / 2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
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
