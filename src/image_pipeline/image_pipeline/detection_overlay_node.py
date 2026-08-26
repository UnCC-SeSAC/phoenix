#!/usr/bin/env python3
"""검출 오버레이 노드 — **박스가 구워진 영상 토픽**을 냅니다 (rqt용).

  구독  /image_enhanced   (sensor_msgs/Image)            <- yolo_node가 받는 그 영상
  구독  /yolo_result      (vision_msgs/Detection2DArray) <- yolo_node
  발행  /yolo/overlay             (sensor_msgs/Image, bgr8)
  발행  /yolo/overlay/compressed  (sensor_msgs/CompressedImage, jpeg)

  ros2 run image_pipeline detection_overlay_node --ros-args \\
      -p image_topic:=/ascamera/camera_publisher/rgb0/image

`yolo_node`는 `Detection2DArray`만 냅니다. rqt_image_view는 그걸 못 그리므로
"모델이 실제로 뭘 보는지"를 눈으로 확인할 방법이 없었습니다. 기존 수단 둘은
둘 다 rqt로 못 봅니다 — `tools/live_detection_preview.py`는 `cv2.imshow`(로봇에
디스플레이 필요), `grpc_bbox_viewer.py`는 gRPC 서버를 따로 띄워야 합니다.
이 노드는 **평범한 Image 토픽 하나**로 끝내서 원격 PC의 rqt에서 바로 열립니다.

그리기는 전부 `image_pipeline/overlay.py`에 있습니다. 이 파일은 배선만 합니다.

★ 반드시 지켜야 하는 것 3가지
------------------------------
1. **`image_topic`은 yolo_node의 `input_topic`과 같아야 합니다.** 박스 좌표는
   YOLO가 받은 프레임 기준입니다. 원본 rgb0에 `/image_enhanced` 기준 박스를
   그리면(전처리가 축소본을 낼 때) 배율만큼 어긋나고, 그 화면을 보고 모델을
   의심하게 됩니다.

2. **구독은 `qos_profile_sensor_data` 호환.** 정수 10을 쓰면 RELIABLE이 되어
   카메라/`yolo_node` 구독이 **조용히** 실패합니다 (`yolo_node._sensor_qos`와 동일).

3. **발행은 기본 RELIABLE.** rqt_image_view가 RELIABLE로 구독하면 BEST_EFFORT
   발행과 호환이 안 되어 창이 영원히 빕니다. 반대 조합(RELIABLE 발행 ↔
   BEST_EFFORT 구독)은 항상 붙으므로 이쪽이 안전합니다. 무선이 나쁘면
   `qos_reliability:=best_effort`.

⚠ 이건 디버그 전용입니다
------------------------
미션 경로(`detection_3d_node` -> `/fire/detections`)와 무관한 순수 소비자입니다.
JPEG 인코딩이 CPU를 먹으므로 `max_fps`(기본 10)로 스로틀합니다. 파이 5에서
추론과 코어를 다투게 하고 싶지 않으면 `display_width`를 더 낮추세요.
"""

from __future__ import annotations

import time
from collections import OrderedDict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2DArray

from image_pipeline.overlay import draw_detections, draw_hud, scale_frame


def stamp_key(header) -> int:
    """`header.stamp` -> ns 정수. 정확 매칭의 열쇠입니다."""
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


class DetectionOverlayNode(Node):

    def __init__(self):
        super().__init__("detection_overlay_node")
        self._declare_params()
        p = self.get_parameter

        self.min_score = float(p("min_score").value)
        self.display_width = int(p("display_width").value)
        self.jpeg_quality = int(p("jpeg_quality").value)
        self.sync_mode = str(p("sync_mode").value).strip().lower()
        self.buffer_size = max(1, int(p("buffer_size").value))
        self.stats_period = float(p("stats_period_sec").value)

        max_fps = float(p("max_fps").value)
        self.min_period = 1.0 / max_fps if max_fps > 0 else 0.0

        image_topic = str(p("image_topic").value)
        det_topic = str(p("detections_topic").value)
        out_topic = str(p("output_topic").value).rstrip("/")

        self.bridge = CvBridge()

        sub_qos = _sensor_qos(int(p("qos_depth").value))
        pub_qos = _pub_qos(str(p("qos_reliability").value),
                           int(p("qos_depth").value))

        self.pub_raw = None
        self.pub_jpeg = None
        if bool(p("publish_raw").value):
            self.pub_raw = self.create_publisher(Image, out_topic, pub_qos)
        if bool(p("publish_compressed").value):
            self.pub_jpeg = self.create_publisher(
                CompressedImage, f"{out_topic}/compressed", pub_qos)
        if self.pub_raw is None and self.pub_jpeg is None:
            raise RuntimeError(
                "publish_raw 와 publish_compressed 가 둘 다 false 입니다 — "
                "아무것도 발행하지 않는 노드가 됩니다")

        # stamp -> BGR 프레임. 검출이 도착하면 pop 합니다.
        self._frames: OrderedDict[int, np.ndarray] = OrderedDict()
        self._latest = None            # sync_mode=latest 용

        self.create_subscription(Image, image_topic, self.on_image, sub_qos)
        self.create_subscription(Detection2DArray, det_topic, self.on_detections,
                                 sub_qos)

        # --- 통계 ---
        self._n_img = 0
        self._n_det_msg = 0
        self._n_pub = 0
        self._n_unmatched = 0
        self._n_throttled = 0
        self._last_pub = 0.0
        self._last_report = time.monotonic()
        self._fps = 0.0
        self._fps_t = None

        self.create_timer(max(1.0, self.stats_period), self._report)

        self.get_logger().info(
            f"[오버레이] {image_topic} + {det_topic} -> {out_topic}"
            f"{' (+/compressed)' if self.pub_jpeg is not None else ''} | "
            f"매칭={self.sync_mode} | 최대 {max_fps:.1f}fps | "
            f"폭={self.display_width or '원본'} | conf>={self.min_score:.2f}")
        self.get_logger().info(
            "★ image_topic 은 yolo_node 의 input_topic 과 같아야 박스가 맞습니다")

    def _declare_params(self):
        # ★ yolo_node 와 같은 기본값. 전처리를 끼운 실제 배포 배선입니다.
        self.declare_parameter("image_topic", "/image_enhanced")
        self.declare_parameter("detections_topic", "/yolo_result")
        self.declare_parameter("output_topic", "/yolo/overlay")

        # exact: stamp가 같은 프레임에만 그립니다 (yolo_node가 헤더를 그대로
        #        복사하므로 정상 배선에서는 항상 맞습니다).
        # latest: stamp가 어긋나는 비정상 상황(bag 재생, 중간에 낀 노드)에서
        #        최신 프레임에 최신 검출을 겹쳐 보여주는 비상용. 박스가 한
        #        프레임 늦을 수 있습니다.
        self.declare_parameter("sync_mode", "exact")
        self.declare_parameter("buffer_size", 30)

        self.declare_parameter("min_score", 0.0)   # 낮은 confidence도 보여줍니다
        self.declare_parameter("display_width", 640)   # 0이면 원본
        self.declare_parameter("max_fps", 10.0)
        self.declare_parameter("jpeg_quality", 75)

        self.declare_parameter("publish_raw", True)
        self.declare_parameter("publish_compressed", True)
        self.declare_parameter("qos_reliability", "reliable")
        self.declare_parameter("qos_depth", 1)
        self.declare_parameter("stats_period_sec", 5.0)

    # -------------------------------------------------------------------- main

    def on_image(self, msg: Image):
        self._n_img += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge 변환 실패: {e}")
            return

        if self.sync_mode == "latest":
            self._latest = (msg.header, frame)
            return

        self._frames[stamp_key(msg.header)] = frame
        while len(self._frames) > self.buffer_size:
            self._frames.popitem(last=False)

    def on_detections(self, msg: Detection2DArray):
        self._n_det_msg += 1

        if self.sync_mode == "latest":
            if self._latest is None:
                self._n_unmatched += 1
                return
            header, frame = self._latest
        else:
            key = stamp_key(msg.header)
            frame = self._frames.pop(key, None)
            if frame is None:
                # 영상보다 검출이 먼저 왔거나 stamp가 어긋난 것. 세어 둡니다 —
                # 화면이 비는 이유를 로그만 보고 가릴 수 있어야 합니다.
                self._n_unmatched += 1
                return
            header = msg.header

        # ★ 스로틀 **전에** 잽니다. 발행 주기를 재면 항상 max_fps 가 나와서
        #   "추론이 밀리는가"라는 정작 궁금한 것을 못 봅니다.
        now = time.monotonic()
        self._tick_fps(now)

        if self.min_period > 0.0 and (now - self._last_pub) < self.min_period:
            self._n_throttled += 1
            return

        # ★ 원본 버퍼를 그대로 그리면 latest 모드에서 같은 프레임에 박스가
        #   계속 덧칠됩니다. 복사본에 그립니다.
        canvas = frame.copy()
        n_drawn = draw_detections(canvas, msg.detections,
                                  min_score=self.min_score)
        draw_hud(canvas, n_drawn=n_drawn, n_total=len(msg.detections),
                 fps=self._fps,
                 note=f"conf>={self.min_score:.2f}" if self.min_score > 0 else "")

        canvas = scale_frame(canvas, self.display_width)
        self._publish(header, canvas)

        self._last_pub = now
        self._n_pub += 1

    def _publish(self, header, canvas):
        if self.pub_raw is not None:
            out = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
            # ★ 원본 stamp를 그대로 실어 나릅니다. rqt는 신경 안 쓰지만,
            #   bag으로 떠서 나중에 검출 로그와 맞춰 볼 때 이게 없으면 못 맞춥니다.
            out.header = header
            self.pub_raw.publish(out)

        if self.pub_jpeg is not None:
            ok, buf = cv2.imencode(
                ".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not ok:
                self.get_logger().warn("JPEG 인코딩 실패")
                return
            jpeg = CompressedImage()
            jpeg.header = header
            jpeg.format = "jpeg"
            jpeg.data = buf.tobytes()
            self.pub_jpeg.publish(jpeg)

    def _tick_fps(self, now: float):
        if self._fps_t is not None:
            dt = now - self._fps_t
            if dt > 0:
                # EMA — 순간값은 튀어서 화면에서 읽기 어렵습니다.
                self._fps = 0.8 * self._fps + 0.2 * (1.0 / dt)
        self._fps_t = now

    # ------------------------------------------------------------------- stats

    def _report(self):
        now = time.monotonic()
        elapsed = now - self._last_report
        if elapsed <= 0:
            return
        msg = (f"[오버레이] 영상 {self._n_img / elapsed:4.1f}Hz | "
               f"검출메시지 {self._n_det_msg / elapsed:4.1f}Hz | "
               f"발행 {self._n_pub}건")
        if self._n_throttled:
            msg += f" | 스로틀 {self._n_throttled}"
        if self._n_unmatched:
            msg += f" | 매칭실패 {self._n_unmatched}"

        if self._n_img and not self._n_det_msg:
            msg += ("  <-- ★ 영상은 오는데 검출 토픽이 없습니다. yolo_node 가 "
                    "떴는지 / detections_topic 이름이 맞는지 확인하세요")
        elif not self._n_img:
            msg += ("  <-- ★ 영상이 안 옵니다. image_topic 이름과 카메라를 "
                    "확인하세요 (RGB는 /image, 뎁스는 /image_raw 입니다)")
        elif self._n_unmatched and not self._n_pub:
            msg += ("  <-- ★ 둘 다 오는데 stamp 가 하나도 안 맞습니다. "
                    "image_topic 이 yolo_node 의 input_topic 과 같은지 확인하고, "
                    "아니면 sync_mode:=latest 로 우회하세요")

        self.get_logger().info(msg)
        self._n_img = self._n_det_msg = self._n_pub = 0
        self._n_unmatched = self._n_throttled = 0
        self._last_report = now


def _sensor_qos(depth: int) -> QoSProfile:
    """`qos_profile_sensor_data`와 **호환되면서** 큐 깊이만 바꿉니다.

    reliability/durability/history를 그대로 베끼는 게 핵심입니다. 하나라도
    다르면 구독이 조용히 안 붙습니다 (`yolo_node._sensor_qos`와 동일).
    """
    qos = QoSProfile(depth=max(1, int(depth)))
    qos.reliability = qos_profile_sensor_data.reliability
    qos.durability = qos_profile_sensor_data.durability
    qos.history = qos_profile_sensor_data.history
    return qos


def _pub_qos(reliability: str, depth: int) -> QoSProfile:
    qos = QoSProfile(depth=max(1, int(depth)))
    qos.reliability = (
        QoSReliabilityPolicy.BEST_EFFORT
        if str(reliability).strip().lower() == "best_effort"
        else QoSReliabilityPolicy.RELIABLE)
    return qos


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DetectionOverlayNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
