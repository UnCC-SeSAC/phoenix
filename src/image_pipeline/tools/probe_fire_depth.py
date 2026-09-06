#!/usr/bin/env python3
"""fire 박스 **주변의 뎁스가 실제로 어떻게 생겼는지** 눈으로 봅니다.

2026-09-06: `below`(박스 바로 아래 1px)는 유효 픽셀 0건, `bottom`(박스 안
아래쪽)은 촛대 사이로 **벽**(1.35m)을 잡았습니다. 둘 다 "어디에 유효 픽셀이
있는가"를 모르는 채 고른 위치라 추측이 반복됐습니다. 이 도구는 그 추측을
끝내려고 있습니다 — 샘플링 위치를 바꾸기 **전에** 돌리세요.

    ros2 run image_pipeline probe_fire_depth        # 또는
    /usr/bin/python3 tools/probe_fire_depth.py

두 가지를 냅니다:
  1. 박스 주변 ASCII 맵 — 구멍의 **모양과 크기**
  2. 아래로 내려가며 gap별 유효/거리 표 — "몇 px 아래로 가면 되나"에 직답
"""
from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray

from image_pipeline.depth import clip_box, project_box, to_meters

# 거리 -> 문자. 벽과 대상을 **눈으로 갈라야** 하므로 10cm 단위입니다.
_RAMP = "0123456789abcdefghijklmnopqrstuvwxyz"


def _cell(v: float) -> str:
    if not np.isfinite(v):
        return "."
    i = int(v / 0.1)
    return _RAMP[i] if 0 <= i < len(_RAMP) else "+"


class Probe(Node):
    def __init__(self):
        super().__init__("probe_fire_depth")
        self.declare_parameter("depth_topic", "/ascamera/camera_publisher/depth0/image_raw")
        self.declare_parameter("depth_info_topic", "/ascamera/camera_publisher/depth0/camera_info")
        self.declare_parameter("color_info_topic", "/image_enhanced/camera_info")
        self.declare_parameter("detections_topic", "/yolo_result")
        self.declare_parameter("class_name", "fire")
        self.declare_parameter("period_sec", 2.0)

        p = self.get_parameter
        self.cls = str(p("class_name").value)
        self.period = float(p("period_sec").value)
        self.bridge = CvBridge()
        self.k_color = self.k_depth = None
        self._last = 0.0

        self.create_subscription(CameraInfo, str(p("color_info_topic").value),
                                 lambda m: setattr(self, "k_color", np.array(m.k).reshape(3, 3)), 10)
        self.create_subscription(CameraInfo, str(p("depth_info_topic").value),
                                 lambda m: setattr(self, "k_depth", np.array(m.k).reshape(3, 3)), 10)
        sync = ApproximateTimeSynchronizer(
            [Subscriber(self, Detection2DArray, str(p("detections_topic").value)),
             Subscriber(self, Image, str(p("depth_topic").value))],
            queue_size=30, slop=0.05)
        sync.registerCallback(self.on_synced)
        self.get_logger().info("probe_fire_depth 시작 — fire 박스를 기다립니다")

    def on_synced(self, det_msg, depth_msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last < self.period or None in (self.k_color, self.k_depth):
            return

        raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        depth_m = to_meters(raw, encoding=depth_msg.encoding)
        H, W = depth_m.shape[:2]

        for det in det_msg.detections:
            if not det.results:
                continue
            if str(det.results[0].hypothesis.class_id) != self.cls:
                continue
            c = det.bbox.center
            box = (c.position.x - det.bbox.size_x / 2, c.position.y - det.bbox.size_y / 2,
                   c.position.x + det.bbox.size_x / 2, c.position.y + det.bbox.size_y / 2)
            self._report(depth_m, project_box(box, self.k_color, self.k_depth), W, H)
            self._last = now
            return

    def _report(self, depth_m, box_d, W, H):
        x1, y1, x2, y2 = (float(v) for v in box_d)
        bw, bh = x2 - x1, y2 - y1
        lines = [f"\n{'='*72}",
                 f"fire 박스(뎁스좌표) x {x1:.0f}..{x2:.0f}  y {y1:.0f}..{y2:.0f}"
                 f"  ({bw:.0f}x{bh:.0f}px)  이미지 {W}x{H}"]

        # --- 1. ASCII 맵: 박스 위아래로 넉넉히
        mx1, mx2 = x1 - bw * 0.5, x2 + bw * 0.5
        my1, my2 = y1 - bh * 0.3, y2 + bh * 2.0
        m = clip_box((mx1, my1, mx2, my2), W, H)
        if m is not None:
            gx1, gy1, gx2, gy2 = m
            sub = depth_m[gy1:gy2, gx1:gx2]
            cols, rows = min(64, gx2 - gx1), min(28, gy2 - gy1)
            xs = np.linspace(0, sub.shape[1] - 1, cols).astype(int)
            ys = np.linspace(0, sub.shape[0] - 1, rows).astype(int)
            lines += ["", "  '.'=무효, 0~9/a~z = 거리 10cm 단위 (예 'd'=1.3~1.4m)",
                      f"  맵 범위 x {gx1}..{gx2}  y {gy1}..{gy2}", ""]
            for yy in ys:
                row = "".join(_cell(float(sub[yy, xx])) for xx in xs)
                mark = "<-박스아랫변" if abs((gy1 + yy) - y2) <= (gy2 - gy1) / rows else ""
                lines.append(f"  y{gy1 + yy:4d} |{row}| {mark}")

        # --- 2. gap 표: "몇 px 아래로 가면 되나"에 직답
        lines += ["", f"  {'gap':>4} {'y':>5} {'유효/전체':>11} {'중앙값':>8} {'최소':>7} {'최대':>7}"]
        cx, hw = (x1 + x2) / 2.0, bw / 2.0
        for gap in (0, 1, 2, 3, 5, 8, 12, 18, 25, 35, 50):
            b = clip_box((cx - hw, y2 + gap, cx + hw, y2 + gap + 1), W, H)
            if b is None:
                lines.append(f"  {gap:>4} {y2 + gap:5.0f}  화면 밖")
                continue
            v = depth_m[b[1]:b[3], b[0]:b[2]].reshape(-1)
            g = v[np.isfinite(v)]
            if g.size:
                lines.append(f"  {gap:>4} {y2 + gap:5.0f} {g.size:5d}/{v.size:<5d} "
                             f"{np.median(g):8.3f} {g.min():7.3f} {g.max():7.3f}")
            else:
                lines.append(f"  {gap:>4} {y2 + gap:5.0f} {0:5d}/{v.size:<5d}  (전부 무효)")
        self.get_logger().info("\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = Probe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
