#!/usr/bin/env python3
"""
YOLO26 검출 노드 — 태스크①의 출력을 받아 태스크②의 입력을 만듭니다.

  구독  /image_enhanced   (sensor_msgs/Image, bgr8)      <- preprocess_node
  발행  /yolo_result      (vision_msgs/Detection2DArray) -> detection_3d_node

  ros2 run image_pipeline yolo_node --ros-args \\
      -p model_path:=models/fire_yolo26s.onnx \\
      -p class_names:="['fire','person']"

`fake_detection_node`가 흉내 내던 자리를 그대로 대체합니다. 박스 좌표계도
같습니다 — **받은 이미지 그대로**(즉 `/image_enhanced`가 축소본이면 축소본
좌표). 원본 `rgb0`로 되돌리는 것은 태스크②의 몫입니다. 여기서 미리 되돌리면
두 번 되돌아가 배율만큼 틀립니다.

추론은 전부 `image_pipeline/yolo.py`에 있습니다. 이 파일은 배선만 합니다.

★ 반드시 지켜야 하는 것 3가지
------------------------------
1. **stamp 전파.** `arr.header = msg.header` — 발행 시각(`now()`)을 넣으면
   태스크②의 `ApproximateTimeSynchronizer`가 뎁스와 영원히 안 맞거나,
   더 나쁘게는 **가장 가까운 엉뚱한 뎁스와 맞아** 회전 중 3.2m에서 16cm가
   조용히 틀립니다 (HANDOVER 4-8, 5-2d).
   2026-08-10 역할 이관으로 이건 **남과의 계약이 아니라 내부 불변식**이
   됐습니다 — 검토자가 없으니 더 위험합니다. 태스크②의 `StampMonitor`가
   런타임에 감시하고, 여기서는 `_publish`가 한 줄로 지킵니다.

2. **검출이 0개여도 발행합니다** (`publish_empty`, 기본 True).
   태스크②의 하트비트는 "마지막으로 **처리한** 프레임"의 나이로 정상/멈춤을
   가릅니다(`detection_3d_node._heartbeat`). 검출이 없다고 여기서 안 내면
   동기화 자체가 안 돌아서, **불이 없는 정상 상태가 '멈춤'으로 보고**됩니다.
   태스크②가 "검출 0개면 발행 안 함"인 것과 혼동하지 마세요 — 그건 메인에
   나가는 이벤트 토픽 이야기이고, 이 노드는 파이프라인 내부입니다.

3. **`qos_profile_sensor_data`.** 정수 10을 쓰면 RELIABLE이 되어 센서 구독이
   조용히 실패합니다.

⚠ 입력 큐는 기본 1입니다 (`qos_depth`)
--------------------------------------
추론이 15fps보다 느리면 큐가 깊을수록 **오래된 프레임**을 붙잡고 처리해서
지연이 계속 쌓입니다. 로봇이 움직이는 중이라 낡은 프레임은 값이 없습니다.
큐 1은 "밀리면 버린다"는 뜻이고, 버린 개수는 `[perf]` 로그에 나옵니다.
버림이 계속 보이면 `imgsz`를 낮추거나(640 -> 480) 모델을 n으로 내리세요.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from image_pipeline.detection_msgs import set_bbox_center, set_hypothesis
from image_pipeline.yolo import make_detector


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        self._declare_params()
        p = self.get_parameter

        model_path = str(p("model_path").value).strip()
        backend = str(p("backend").value).strip() or "auto"
        if backend == "stub":
            # ★ 배선 확인 전용. 가중치를 읽지 않고 화면 정중앙에 박스를 놓습니다.
            #   **명시적으로 골라야만** 켜집니다 — 모델이 없을 때 자동으로
            #   여기로 빠지면 "검출이 이상한 노드"가 되어 원인을 못 찾습니다.
            self.get_logger().warn(
                "backend=stub — 가중치를 읽지 않습니다. 화면 정중앙에 박스 하나를 "
                "놓을 뿐이고 **검출이 아닙니다.** 배선(QoS·stamp·좌표) 확인용입니다.")
        elif not model_path:
            # ★ 조용히 놀지 않습니다. 모델 없이 뜬 노드는 "검출이 하나도 안 나오는
            #   정상 노드"처럼 보여서, 원인을 태스크② 쪽에서 찾게 만듭니다.
            raise RuntimeError(
                "model_path 가 비어 있습니다.\n"
                "  학습된 모델이 있으면:  -p model_path:=models/fire_yolo26s.onnx\n"
                "  아직 없으면 더미로 태스크②를 돌리세요:\n"
                "    ros2 run image_pipeline fake_detection_node")

        # 빈 문자열은 걸러냅니다 — rclpy는 빈 리스트를 기본값으로 못 받아서
        # 기본값이 [""] 입니다. 안 거르면 클래스가 1개 있는 걸로 오인됩니다.
        self.class_names = [str(n) for n in (p("class_names").value or [])
                            if str(n).strip()]
        if not self.class_names:
            self.get_logger().warn(
                "class_names 가 비어 있습니다 — class_name 이 번호 문자열('0')로 "
                "나갑니다. 메인은 'fire'를 기대합니다. **학습 때 순서 그대로** "
                "넣으세요 (순서가 틀리면 불을 사람으로 발행합니다).")

        self.publish_empty = bool(p("publish_empty").value)
        self.max_age = float(p("max_input_age_sec").value)
        self.stats_period = float(p("stats_period_sec").value)

        # 추론 본체. ROS를 모르는 모듈이라 tests/ 에서 같은 코드가 검증됩니다.
        self.detector = make_detector(
            model_path,
            backend=backend,
            imgsz=int(p("imgsz").value),
            conf=float(p("conf").value),
            iou=float(p("iou").value),
            class_names=self.class_names,
            layout=str(p("layout").value),
            normalized=str(p("normalized").value),
            agnostic=bool(p("agnostic_nms").value),
            max_det=int(p("max_det").value),
            swap_rb=bool(p("swap_rb").value),
            threads=int(p("threads").value),
        )

        self.bridge = CvBridge()

        in_topic = str(p("input_topic").value)
        out_topic = str(p("detections_topic").value)
        qos = _sensor_qos(int(p("qos_depth").value))

        self.pub = self.create_publisher(Detection2DArray, out_topic, qos)
        self.sub = self.create_subscription(Image, in_topic, self.on_image, qos)

        # --- 통계 ---
        self._n_in = 0
        self._n_pub = 0
        self._n_det = 0
        self._n_dropped = 0
        self._n_failed = 0
        self._t_infer: list[float] = []
        self._last_report = time.monotonic()

        self.add_on_set_parameters_callback(self.on_param_update)

        self.get_logger().info(
            f"[YOLO] {in_topic} -> {out_topic} | {model_path} "
            f"| imgsz={self.detector.imgsz} conf={self.detector.conf:.2f} "
            f"iou={self.detector.iou:.2f} | 레이아웃={self.detector.detected_layout} "
            f"| 클래스={self.class_names or '(번호)'}")
        if str(p("layout").value) == "auto":
            # ★ 자동 판별은 휴리스틱입니다. 실제 모델이 오면 눈으로 확인하고
            #   못박으라는 안내를 시작 로그에 남깁니다 (yolo.py 문서 참조).
            self.get_logger().info(
                "레이아웃 자동 판별 중입니다. 실제 모델을 처음 붙였다면 "
                "describe_outputs() 로 실제 shape를 확인하고 layout 을 못박으세요.")

    # ------------------------------------------------------------------ params

    def _declare_params(self):
        # 태스크①의 출력. 원본 rgb0 가 아니라 **전처리된** 영상을 봅니다 —
        # 디헤이즈·CLAHE가 검출률을 올리려고 존재하는 것이니까요.
        self.declare_parameter("input_topic", "/image_enhanced")
        # 태스크②의 기본 구독 이름과 같아야 합니다 (detection_3d_node).
        self.declare_parameter("detections_topic", "/yolo_result")

        self.declare_parameter("model_path", "")
        # auto|onnx|ultralytics|hailo|stub. stub은 **배선 확인 전용** —
        # 가중치를 안 읽고 화면 정중앙에 박스를 놓습니다 (검출이 아닙니다).
        self.declare_parameter("backend", "auto")
        self.declare_parameter("imgsz", 640)        # ★ 학습 때 값과 같아야 함
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("max_det", 300)
        # ★ 학습 때 순서 그대로. 순서가 틀리면 'fire'를 'person'으로 발행합니다.
        self.declare_parameter("class_names", [""])
        # "auto"|"v8"|"end2end" — 실제 모델 확인 후 못박으세요 (HANDOVER 7-2)
        self.declare_parameter("layout", "auto")
        self.declare_parameter("normalized", "auto")   # auto|no
        self.declare_parameter("agnostic_nms", False)
        # 우리 파이프라인은 bgr8, ultralytics 학습은 RGB. 끄면 불꽃이 파랗게 들어갑니다.
        self.declare_parameter("swap_rb", True)
        # Pi 5(4코어)에서 ROS·Hailo 드라이버와 코어를 나눠 씁니다. 0이면 건드리지 않음.
        self.declare_parameter("threads", 0)

        # ★ 기본 1 = 밀리면 낡은 프레임을 버립니다. 위 문서의 「입력 큐」 참조.
        self.declare_parameter("qos_depth", 1)
        # ★ 기본 True. 태스크② 하트비트가 이것에 의존합니다 (위 문서 2번).
        self.declare_parameter("publish_empty", True)
        # 0 = 끔. >0 이면 그보다 낡은 프레임은 추론을 건너뜁니다.
        # 켤 때 주의: 카메라 stamp와 노드 시계가 다른 소스면(하드웨어 타임스탬프)
        # 멀쩡한 프레임을 전부 버립니다. 로봇에서 실측 후 켜세요.
        self.declare_parameter("max_input_age_sec", 0.0)
        self.declare_parameter("stats_period_sec", 5.0)

    def on_param_update(self, params):
        """`conf`/`iou`는 현장에서 돌려가며 맞춰야 하는 값이라 런타임 변경을 엽니다."""
        for prm in params:
            if prm.name == "conf":
                self.detector.conf = float(prm.value)
            elif prm.name == "iou":
                self.detector.iou = float(prm.value)
            elif prm.name == "max_det":
                self.detector.max_det = int(prm.value)
            elif prm.name == "publish_empty":
                self.publish_empty = bool(prm.value)
            elif prm.name == "max_input_age_sec":
                self.max_age = float(prm.value)
        return SetParametersResult(successful=True)

    # -------------------------------------------------------------------- main

    def on_image(self, msg: Image):
        self._n_in += 1

        if self.max_age > 0.0 and self._age_sec(msg) > self.max_age:
            # 밀린 프레임. 처리해봐야 로봇은 이미 다른 곳을 보고 있습니다.
            self._n_dropped += 1
            self._report_if_due()
            return

        try:
            # ★ encoding 명시. 컬러이므로 bgr8 — yolo.make_blob 이 RGB로 뒤집습니다.
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self._n_failed += 1
            self.get_logger().error(f"cv_bridge 변환 실패: {e}")
            return

        try:
            dets = self.detector.detect(img)
        except Exception as e:  # noqa: BLE001
            # ★ 예외를 삼키지 않습니다. 삼키면 "검출이 안 되는 노드"로 보여서
            #   원인을 모델이 아니라 전처리에서 찾게 됩니다.
            self._n_failed += 1
            self.get_logger().error(
                f"추론 실패: {e} — 출력 레이아웃/클래스 수를 확인하세요 "
                "(yolo.describe_outputs)")
            self._report_if_due()
            return

        self._t_infer.append(self.detector.timings["total"])
        self._n_det += len(dets)
        self._publish(msg, dets)
        self._report_if_due()

    def _publish(self, src: Image, dets):
        if not dets and not self.publish_empty:
            return

        arr = Detection2DArray()
        # ★★ 이 한 줄이 stamp 와 frame_id 를 동시에 실어 나릅니다.
        #    now() 로 덮으면 태스크②의 동기화가 통째로 깨집니다 (문서 1번).
        arr.header = src.header

        for d in dets:
            det = Detection2D()
            det.header = src.header
            u, v = d.center()
            w, h = d.size()
            set_bbox_center(det.bbox, u, v)
            # ★ size_x/size_y 는 **크기**입니다. 우하단 좌표가 아닙니다
            #   (detection_msgs.box_from_bbox 가 다시 (x1,y1,x2,y2)로 되돌립니다).
            det.bbox.size_x = float(w)
            det.bbox.size_y = float(h)

            result = ObjectHypothesisWithPose()
            set_hypothesis(result, d.class_name or str(d.class_id), d.score)
            det.results.append(result)
            arr.detections.append(det)

        self.pub.publish(arr)
        self._n_pub += 1

    def _age_sec(self, msg: Image) -> float:
        now = self.get_clock().now().to_msg()
        return ((now.sec - msg.header.stamp.sec) +
                (now.nanosec - msg.header.stamp.nanosec) * 1e-9)

    # ------------------------------------------------------------------- stats

    def _report_if_due(self):
        now = time.monotonic()
        if now - self._last_report < self.stats_period:
            return
        elapsed = now - self._last_report
        in_hz = self._n_in / elapsed if elapsed > 0 else 0.0

        msg = (f"[YOLO] 입력 {in_hz:4.1f}Hz | 발행 {self._n_pub}건 "
               f"| 검출 {self._n_det}개")
        anomaly = False
        if self._t_infer:
            mean = float(np.mean(self._t_infer))
            p95 = float(np.percentile(self._t_infer, 95))
            msg += f" | 추론 평균 {mean:6.2f}ms (p95 {p95:6.2f}ms)"
            budget = 1000.0 / in_hz if in_hz > 0 else float("inf")
            if mean > budget:
                msg += (f"  <-- ★ 입력 주기 {budget:.1f}ms 초과. 프레임이 밀립니다. "
                        "imgsz를 낮추거나 모델을 n으로 내리세요")
                anomaly = True
        if self._n_dropped:
            msg += f" | 낡아서 버림 {self._n_dropped}"
        if self._n_failed:
            msg += f" | 실패 {self._n_failed}"
        if self._n_in and not self._n_pub:
            msg += "  <-- 입력은 오는데 하나도 발행하지 않았습니다"
            anomaly = True

        # 정상 범위에서는 확인용 주기 로그라 debug로. 병목/미발행 등 실제
        # 문제가 있을 때만 warn으로 남깁니다.
        if anomaly:
            self.get_logger().warn(msg)
        else:
            self.get_logger().debug(msg)
            
        self._n_in = self._n_pub = self._n_det = 0
        self._n_dropped = self._n_failed = 0
        self._t_infer.clear()
        self._last_report = now


def _sensor_qos(depth: int) -> QoSProfile:
    """`qos_profile_sensor_data`와 **호환되면서** 큐 깊이만 바꿉니다.

    reliability/durability/history를 그대로 베끼는 게 핵심입니다. 하나라도
    다르면 구독이 조용히 안 붙습니다 (`detection_3d_node._sensor_qos`와 동일).
    """
    qos = QoSProfile(depth=max(1, int(depth)))
    qos.reliability = qos_profile_sensor_data.reliability
    qos.durability = qos_profile_sensor_data.durability
    qos.history = qos_profile_sensor_data.history
    return qos


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloNode()
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
