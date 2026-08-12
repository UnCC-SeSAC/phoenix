#!/usr/bin/env python3
"""
`yolo_node` 배선 확인 — **가중치 없이** ROS 층만 검증합니다.

    source /opt/ros/jazzy/setup.bash
    export PYTHONPATH=$PWD:$PYTHONPATH
    python3 tools/check_yolo_wiring.py

pytest(`tests/test_yolo.py`)는 ROS를 안 켜므로 **QoS와 헤더 전파를 못 봅니다.**
그 둘이 이 저장소에서 가장 조용하게 깨지는 곳이라, 한 프로세스에 노드 3개를
띄워 직접 확인합니다:

    [발행] /image_enhanced ──▶ [yolo_node + 스텁 검출기] ──▶ /yolo_result ──▶ [검사]

보는 것 4가지:

  1. **QoS 매칭** — `Detection2DArray`가 실제로 도착하는가.
     `qos_profile_sensor_data` 대신 정수 10을 쓰면 여기서 0건이 됩니다.
  2. **stamp 전파** — 발행한 이미지의 stamp가 그대로 실려 오는가.
     `now()`로 덮으면 태스크②의 동기화가 깨집니다 (HANDOVER 4-8).
  3. **frame_id 전파** — 같은 이유.
  4. **박스 왕복** — `(x1,y1,x2,y2)` → center+size → 다시 `(x1,y1,x2,y2)`.
     `size_x`를 `x2`로 오해하면 여기서 어긋납니다.

검출 정확도는 **보지 않습니다.** 그건 가중치가 있어야 합니다.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import image_pipeline.yolo_node as yolo_node  # noqa: E402
from image_pipeline.detection_msgs import box_from_bbox, hypothesis  # noqa: E402
from image_pipeline.yolo import Detection  # noqa: E402

BOX = (100.0, 50.0, 200.0, 150.0)
SCORE = 0.87
CLASS = "fire"
FRAME_ID = "camera_color_optical_frame"
SECONDS = 3.0
FPS = 15.0


class StubDetector:
    """`yolo.StubBackend`보다 더 단순 — 좌표를 고정해 왕복만 봅니다."""

    imgsz, conf, iou, max_det = 640, 0.25, 0.45, 300
    detected_layout = "stub"
    timings = {"pre": 0.0, "infer": 0.0, "post": 0.0, "total": 0.0}

    def detect(self, img):
        return [Detection(BOX, SCORE, 0, CLASS)]


class Publisher(Node):
    """태스크①(`preprocess_node`) 자리를 대신합니다."""

    def __init__(self):
        super().__init__("wiring_check_pub")
        self.pub = self.create_publisher(Image, "/image_enhanced",
                                         qos_profile_sensor_data)
        self.msg = CvBridge().cv2_to_imgmsg(
            np.zeros((480, 640, 3), np.uint8), encoding="bgr8")
        self.msg.header.frame_id = FRAME_ID
        self.sent = 0
        self.stamps: set[tuple[int, int]] = set()
        self.create_timer(1.0 / FPS, self.tick)

    def tick(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.stamps.add((self.msg.header.stamp.sec, self.msg.header.stamp.nanosec))
        self.sent += 1
        self.pub.publish(self.msg)


class Checker(Node):
    """태스크②(`detection_3d_node`) 자리에서 받은 것만 검사합니다."""

    def __init__(self, publisher: Publisher):
        super().__init__("wiring_check_sub")
        self.publisher = publisher
        self.got = 0
        self.problems: list[str] = []
        self.create_subscription(Detection2DArray, "/yolo_result",
                                 self.on_detections, qos_profile_sensor_data)

    def on_detections(self, msg: Detection2DArray):
        self.got += 1
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)

        if stamp == (0, 0):
            self._note("stamp가 0입니다 — 헤더를 복사하지 않았습니다")
        elif stamp not in self.publisher.stamps:
            self._note(f"stamp {stamp} 가 발행한 것 중에 없습니다 — now()로 덮었습니다")
        if msg.header.frame_id != FRAME_ID:
            self._note(f"frame_id가 {msg.header.frame_id!r} 입니다")

        if not msg.detections:
            self._note("검출이 비어 있습니다")
            return
        det = msg.detections[0]
        box = box_from_bbox(det.bbox)
        if not np.allclose(box, BOX, atol=1e-3):
            self._note(f"박스 왕복 실패: {tuple(round(v, 1) for v in box)} != {BOX}")
        if not det.results:
            self._note("results 가 비어 있습니다")
            return
        cid, score = hypothesis(det.results[0])
        if cid != CLASS:
            self._note(f"class_id가 {cid!r} 입니다")
        if abs(score - SCORE) > 1e-3:
            self._note(f"score가 {score} 입니다")

    def _note(self, text: str):
        if text not in self.problems:
            self.problems.append(text)


def main() -> int:
    # ★ 모델을 읽지 않도록 검출기를 갈아끼웁니다. 노드 코드는 그대로 돕니다.
    yolo_node.make_detector = lambda *a, **k: StubDetector()

    # 노드가 model_path 를 요구하므로 값만 넣어 줍니다 (읽지는 않습니다).
    argv = sys.argv[1:]
    if "--ros-args" not in argv:
        argv = argv + ["--ros-args", "-p", "model_path:=stub",
                       "-p", "stats_period_sec:=1.0"]

    rclpy.init(args=[sys.argv[0]] + argv)
    pub = Publisher()
    node = yolo_node.YoloNode()
    chk = Checker(pub)

    executor = rclpy.executors.SingleThreadedExecutor()
    for n in (pub, node, chk):
        executor.add_node(n)

    deadline = pub.get_clock().now().nanoseconds + int(SECONDS * 1e9)
    while rclpy.ok() and pub.get_clock().now().nanoseconds < deadline:
        executor.spin_once(timeout_sec=0.1)

    print()
    print("=" * 46)
    print(f" 발행한 이미지     {pub.sent}장 ({FPS:.0f}Hz x {SECONDS:.0f}초)")
    print(f" 받은 검출 메시지  {chk.got}건")
    if chk.got == 0:
        chk._note("한 건도 못 받았습니다 — QoS 매칭 또는 토픽 이름 문제")
    if chk.problems:
        print(" 문제:")
        for p in chk.problems:
            print(f"   - {p}")
    else:
        print(" 문제 없음 — QoS · stamp · frame_id · 박스 왕복 전부 정상")
    print()
    print(" ※ 검출 정확도는 보지 않습니다. 그건 가중치가 있어야 합니다.")
    print("=" * 46)

    ok = chk.got > 0 and not chk.problems
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
