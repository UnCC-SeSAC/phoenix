#!/usr/bin/env python3
"""
컨트롤센터 UI 스트림 노드 — 영상과 박스를 **따로** 내보냅니다.

  구독  /image_enhanced   (sensor_msgs/Image, bgr8)       <- preprocess_node
        /yolo_result      (vision_msgs/Detection2DArray)  <- yolo_node
  발행  /ui/camera/compressed (sensor_msgs/CompressedImage, jpeg)
        /ui/camera/overlay    (std_msgs/String JSON, 0..1 정규화 박스)

박스를 픽셀에 굽지 않는 이유는 Pi CPU와 유연성입니다. 브라우저가 겹치므로
`stream_max_width`로 화질을 낮춰도 박스는 그대로 맞습니다.

★ 반드시 지켜야 하는 것 3가지
------------------------------
1. **입력 영상은 `/image_enhanced`여야 합니다.** `/yolo_result`의 박스가 그
   좌표계이기 때문입니다(yolo_node docstring). 원본 rgb0를 띄우면 전처리
   축소 배율만큼 박스가 어긋나는데, **에러 없이** 어긋납니다.

2. **`qos_profile_sensor_data`.** 정수 10을 쓰면 RELIABLE이 되어 센서 구독이
   조용히 실패합니다 — 콜백이 아예 안 불립니다.

3. **검출이 0개여도 오버레이를 발행합니다** (`publish_empty`, 기본 True).
   안 내면 브라우저에 **직전 박스가 그대로 남아** 꺼진 불이 계속 타는 것처럼
   보입니다. 이건 관제 화면에서 가장 위험한 종류의 거짓말입니다.

⚠ 프레임은 밀리면 버립니다 (`qos_depth`=1, `stream_fps` throttle).
   관제 화면에 필요한 건 **지금**이지 3초 전이 아닙니다.
"""

from __future__ import annotations

import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
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

        self.class_names = [str(n) for n in (p("class_names").value or [])
                            if str(n).strip()]
        self.max_width = int(p("stream_max_width").value)
        self.quality = int(p("jpeg_quality").value)
        self.fps = float(p("stream_fps").value)
        self.slop = float(p("slop_sec").value)
        self.publish_empty = bool(p("publish_empty").value)
        self.stats_period = float(p("stats_period_sec").value)

        self.bridge = CvBridge()
        self._dets: dict[tuple[int, int], list] = {}   # stamp -> detections
        self._det_cache_size = int(p("detection_cache").value)
        self._last_emit = None
        self._seq = 0
        self._n_in = 0
        self._n_out = 0
        self._n_unmatched = 0
        self._last_report = time.monotonic()

        img_topic = str(p("input_topic").value)
        det_topic = str(p("detections_topic").value)
        self.frame_pub = self.create_publisher(
            CompressedImage, str(p("stream_topic").value), qos_profile_sensor_data)
        self.overlay_pub = self.create_publisher(
            String, str(p("overlay_topic").value), qos_profile_sensor_data)
        self.create_subscription(
            Detection2DArray, det_topic, self.on_detections, qos_profile_sensor_data)
        self.create_subscription(
            Image, img_topic, self.on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f"[UI] {img_topic} + {det_topic} -> {p('stream_topic').value} "
            f"| {self.fps:g}fps, max_width={self.max_width or '원본'}, "
            f"q={self.quality}")

    def _declare_params(self):
        self.declare_parameter("input_topic", "/image_enhanced")
        self.declare_parameter("detections_topic", "/yolo_result")
        self.declare_parameter("stream_topic", "/ui/camera/compressed")
        self.declare_parameter("overlay_topic", "/ui/camera/overlay")
        # ★ 학습 때 순서 그대로. 틀리면 불을 사람으로 표시합니다.
        self.declare_parameter("class_names", [""])
        self.declare_parameter("stream_fps", 8.0)
        self.declare_parameter("stream_max_width", 640)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("slop_sec", 0.1)
        self.declare_parameter("publish_empty", True)
        self.declare_parameter("detection_cache", 30)
        self.declare_parameter("stats_period_sec", 5.0)

    # ------------------------------------------------------------ detections

    def on_detections(self, msg: Detection2DArray):
        self._dets[_stamp_key(msg.header)] = list(msg.detections)
        # dict는 삽입 순서를 지키므로 오래된 것부터 버립니다.
        while len(self._dets) > self._det_cache_size:
            self._dets.pop(next(iter(self._dets)))

    def _match(self, key):
        """프레임 stamp에 맞는 검출. 정확 매칭이 정상 경로입니다.

        yolo_node가 stamp를 전파하므로 보통 정확히 맞습니다. slop 폴백은
        전처리 경로가 바뀌었을 때를 위한 안전망이고, 이게 자주 쓰이면
        `[perf]`의 unmatched가 올라가니 배선을 의심하세요.
        """
        hit = self._dets.get(key)
        if hit is not None:
            return hit
        if not self._dets:
            return None
        target = _stamp_sec(key)
        nearest = min(self._dets, key=lambda k: abs(_stamp_sec(k) - target))
        if abs(_stamp_sec(nearest) - target) <= self.slop:
            return self._dets[nearest]
        return None

    # ----------------------------------------------------------------- image

    def on_image(self, msg: Image):
        self._n_in += 1
        now = time.monotonic()
        if not throttle(self._last_emit, now, self.fps):
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge 변환 실패: {e}")
            return

        src_h, src_w = img.shape[:2]
        key = _stamp_key(msg.header)
        detections = self._match(key)
        if detections is None:
            self._n_unmatched += 1
            if not self.publish_empty:
                return
            detections = []

        boxes = normalize_boxes(detections, src_w, src_h, self.class_names)

        out_w, out_h = stream_size(src_w, src_h, self.max_width)
        if (out_w, out_h) != (src_w, src_h) and out_w > 0:
            img = cv2.resize(img, (out_w, out_h), interpolation=cv2.INTER_AREA)

        ok, buf = cv2.imencode(
            ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            self.get_logger().warn("JPEG 인코딩 실패 — 프레임을 버립니다")
            return

        self._seq += 1
        frame = CompressedImage()
        frame.header = msg.header          # ★ stamp 전파 (now()로 덮지 않습니다)
        frame.format = "jpeg"
        frame.data = buf.tobytes()
        self.frame_pub.publish(frame)

        overlay = String()
        overlay.data = json.dumps({
            "seq": self._seq,
            "stamp_sec": key[0],
            "stamp_nanosec": key[1],
            "width": out_w,
            "height": out_h,
            "boxes": boxes,
        }, ensure_ascii=False)
        self.overlay_pub.publish(overlay)

        self._last_emit = now
        self._n_out += 1
        self._report(now)

    def _report(self, now):
        if self.stats_period <= 0 or now - self._last_report < self.stats_period:
            return
        self.get_logger().info(
            f"[perf] in={self._n_in} out={self._n_out} "
            f"unmatched={self._n_unmatched} (검출 stamp 불일치)")
        self._n_in = self._n_out = self._n_unmatched = 0
        self._last_report = now


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