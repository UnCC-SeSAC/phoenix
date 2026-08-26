#!/usr/bin/env python3
"""
태스크② 노드 — 2D 검출 + 뎁스 → **픽셀 좌표 + 거리**의 JSON.

  /yolo_result                              (Detection2DArray, 컬러 좌표계)
  /ascamera/.../depth0/image_raw            (16UC1, mm)          ─┐
  /ascamera/.../depth0/camera_info                                │
  /image_enhanced/camera_info               (축소 보정 K)          ├─▶ /fire/detections
  /ascamera/.../rgb0/camera_info            (원본 K)             ─┘   /fire/detections/status

★ 2026-08-10 계약 변경: 출력이 `vision_msgs/Detection3DArray`(base_link 3D)에서
  **JSON**으로 바뀌었고 **역투영과 TF 변환은 메인이 합니다** (HANDOVER 7-3).
  그래서 이 노드에는 tf2 조회가 없습니다. 역투영 코드(`convert_frame`,
  `backproject`, `to_base_link`)는 **검증용으로 모듈에 살아 있습니다**
  (메인 결과가 틀렸을 때 우리 거리 문제인지 메인 역투영 문제인지 가릅니다).

**계산은 전부 `detection3d.py`·`depth.py`·`detection_json.py`에 있습니다.**
이 파일은 구독·동기화·발행만 합니다 (`CLAUDE.md` "계산은 모듈에, 노드는 배선만").

이 노드가 막아주는 조용한 실패
------------------------------
1. **QoS** — 정수 10을 쓰면 콜백이 영원히 안 불립니다. 나아가 이미지처럼 큰
   메시지는 `qos_profile_sensor_data`의 기본 큐(5)로도 절반 넘게 흘립니다
   (실측 8.3Hz vs 13.2Hz) — `qos_depth`로 깊이만 늘립니다. reliability는 그대로.
2. **stamp** — 출력 stamp는 **검출 메시지의 stamp**입니다. 발행 시각을 넣으면
   메인이 그만큼 과거를 현재 위치로 찍습니다 (HANDOVER 4-8).
   나아가 그 stamp가 정말 원본인지 `StampMonitor`가 런타임에 감시합니다.
3. **해상도/화각** — 박스는 컬러 좌표계, 거리는 뎁스 좌표계, 발행은 원본 좌표계.
   좌표계가 셋이고 변환 목적지가 다릅니다. 해상도를 하드코딩하지 않습니다.
4. **거리 불명** — 좌표를 지어내지 않습니다. `depth: null` + `unknown`으로
   **명시해서** 보냅니다 (예전엔 뺐지만, 빼면 메인이 검출 존재조차 모릅니다).
5. **침묵** — 검출이 없으면 이벤트를 안 내므로, 살아있다는 말은 하트비트가
   따로 합니다. 그 하트비트는 "살아있음"이 아니라 **마지막 처리 프레임의
   경과 시간**을 실어 보냅니다 — 단순 펄스는 동기화가 죽어도 정상을 주장합니다.

`camera_info`는 동기화에 넣지 않고 **캐시**합니다. K는 변하지 않는데 함께
동기화하면 세트가 안 맞아 버려지는 프레임만 늘어납니다.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from image_pipeline.depth import depth_unit_sanity, to_meters
from image_pipeline.detection3d import (
    SamplingParams,
    StampMonitor,
    convert_frame_pixels,
    parse_region_by_class,
)
from image_pipeline.detection_json import (
    STATUS_UNKNOWN,
    build_heartbeat,
    build_payload,
    heartbeat_state,
    is_surrogate,
    stamp_age_sec,
    to_json,
)
from image_pipeline.detection_msgs import box_from_bbox, hypothesis
from image_pipeline.intrinsics import principal_point_sanity


class Detection3DNode(Node):
    def __init__(self):
        super().__init__("detection_3d_node")

        self._declare_params()
        p = self.get_parameter

        self.publish_fallback = bool(p("publish_fallback").value)
        self.publish_empty = bool(p("publish_empty").value)
        self.stall_after_sec = float(p("stall_after_sec").value)

        self.params = SamplingParams(
            region=str(p("region").value),
            method=str(p("method").value),
            central=float(p("central").value),
            band_ratio=float(p("band_ratio").value),
            band_offset=float(p("band_offset").value),
            ring_margin=float(p("ring_margin").value),
            min_valid_ratio=float(p("min_valid_ratio").value),
            min_valid_px=int(p("min_valid_px").value),
            z_min=float(p("z_min").value),
            z_max=float(p("z_max").value),
            max_spread_m=(None if float(p("max_spread_m").value) <= 0
                          else float(p("max_spread_m").value)),
            fallback_regions=tuple(str(p("fallback_regions").value).split(",")) if
            str(p("fallback_regions").value).strip() else (),
            # 빈 문자열 = 영역별 권장값(detection3d.method_for). 문자열을 주면
            # **모든** 폴백 영역에 강제됩니다 — ring 에 "max"는 편향을 키웁니다.
            fallback_method=(str(p("fallback_method").value).strip() or None),
            min_score=float(p("min_score").value),
            region_by_class=parse_region_by_class(
                str(p("region_by_class").value)),
        )

        self.bridge = CvBridge()
        self.k_color = None       # /image_enhanced/camera_info — 박스가 있는 좌표계
        self.k_depth = None       # depth0/camera_info — 거리 샘플링
        self.k_rgb0 = None        # rgb0/camera_info — ★ 발행할 픽셀의 기준
        self.rgb0_size = None     # frame_size 로 나갑니다
        self._color_size = None
        self._depth_size = None
        self._depth_checked = False
        self._last_frame = None   # 마지막으로 **처리한** 원본 프레임 stamp

        self.stamp_monitor = StampMonitor(
            history=int(p("stamp_history").value),
            window=int(p("stamp_window").value),
            min_match_ratio=float(p("stamp_min_match").value),
        )
        self._n_det_seen = 0        # 동기화 전, 도착한 검출 메시지 수
        self._n_depth_seen = 0      # 동기화 전, 도착한 뎁스 메시지 수

        # --- 발행 ---
        # 이벤트 — 검출이 있을 때만.
        self.pub = self.create_publisher(
            String, str(p("output_topic").value), qos_profile_sensor_data)
        # 하트비트 — 검출과 무관하게 주기 발행. "불이 없다"와 "노드가 죽었다"를
        # 가르는 유일한 수단입니다.
        self.status_pub = self.create_publisher(
            String, str(p("status_topic").value), qos_profile_sensor_data)

        # --- camera_info 3개 (작은 메시지라 기본 QoS로 충분) ---
        self.create_subscription(CameraInfo, str(p("color_info_topic").value),
                                 self.on_color_info, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, str(p("depth_info_topic").value),
                                 self.on_depth_info, qos_profile_sensor_data)
        # ★ 메인이 역투영에 쓸 K. 우리가 발행할 x,y 가 이 좌표계여야 합니다.
        self.create_subscription(CameraInfo, str(p("rgb0_info_topic").value),
                                 self.on_rgb0_info, qos_profile_sensor_data)

        # ★ 동기화기를 **거치지 않은** 원본 구독. 동기화된 것만 보면 이미
        #   걸러진 뒤라 "검출은 오는데 짝이 안 맞는" 상황을 못 봅니다.
        self.create_subscription(Detection2DArray, str(p("detections_topic").value),
                                 self.on_detection_seen, qos_profile_sensor_data)

        # ★ 큐 깊이만 늘린 sensor QoS. 614KB 뎁스에 기본값 5는 얕습니다.
        sensor_qos = _sensor_qos(int(p("qos_depth").value))
        det_sub = Subscriber(self, Detection2DArray, str(p("detections_topic").value),
                             qos_profile=sensor_qos)
        depth_sub = Subscriber(self, Image, str(p("depth_topic").value),
                               qos_profile=sensor_qos)
        # 동기화가 안 될 때 **어느 쪽이 안 들어오는지** 알아야 원인을 좁힙니다.
        # (검출이 안 옴 / 뎁스가 안 옴 / 둘 다 오는데 stamp가 안 맞음)
        depth_sub.registerCallback(self._count_depth)
        # 큐는 YOLO 지연 x 입력 Hz 를 덮어야 합니다. 15Hz에 30이면 약 2초.
        self.sync = ApproximateTimeSynchronizer(
            [det_sub, depth_sub],
            queue_size=int(p("queue_size").value),
            slop=float(p("slop_sec").value))
        self.sync.registerCallback(self.on_synced)

        self.add_on_set_parameters_callback(self.on_param_update)

        # --- 통계 ---
        # ★ 두 벌입니다. `_n_*`는 stats 주기마다 0으로 리셋되고(Hz 계산용),
        #   `_n_*_total`은 하트비트가 쓰는 **누적**값입니다. 하나로 합치면
        #   하트비트의 counters가 5초마다 0으로 돌아가 "멈춤"처럼 보입니다.
        self.stats_period = float(p("stats_period_sec").value)
        self._n_in = self._n_out = self._n_det_seen = 0
        self._n_frames_total = self._n_pub_total = 0
        self._n_det_total = self._n_depth_total = 0
        self._n_unknown = self._n_fallback = 0
        self._reasons: dict = {}
        self._t_proc: list = []
        self._last_report = time.monotonic()

        self.create_timer(float(p("heartbeat_period_sec").value), self._heartbeat)

        self.get_logger().info(
            f"[태스크②] {p('detections_topic').value} + {p('depth_topic').value} "
            f"-> {p('output_topic').value} (JSON, 검출이 있을 때만) "
            f"| 상태 -> {p('status_topic').value}")
        # ★ 클래스별 매핑은 **반드시 시작 로그로 확인할 수 있어야** 합니다.
        #   라벨 오타는 예외를 안 냅니다 — 그 클래스만 조용히 기본값으로 재고,
        #   fire가 bottom으로 떨어지면 벽 거리가 나갑니다.
        mapping = ", ".join(
            "{}->{}({})".format(c, r, self.params.region_for(c)[1])
            for c, r in sorted(self.params.region_by_class.items()))
        self.get_logger().info(
            f"[태스크②] 거리 샘플링: 기본 {self.params.region}"
            f"({self.params.method})"
            + (f" | 클래스별 {mapping}" if mapping else " | 클래스별 매핑 없음")
            + "  ← 검출 라벨이 여기 이름과 정확히 같은지 확인하세요")
        if self.params.band_offset > 0.0:
            lo, hi = self.params.band_offset, self.params.band_offset + self.params.band_ratio
            self.get_logger().info(
                f"[태스크②] below 띠 = 박스 아래 {lo:.1f}~{hi:.1f}배 지점 "
                f"(박스높이 배수). 화염 1.75cm 기준 아래 "
                f"{lo * 1.75:.1f}~{hi * 1.75:.1f}cm — 촛대 받침(종이컵)을 겨냥합니다")
        if self.params.fallback_regions:
            self.get_logger().warn(
                f"폴백 켜짐: {self.params.fallback_regions} — 이 값들은 대상이 아니라 "
                "**주변**을 잽니다(below 가깝게 / ring 멀게). 실측으로 검증하세요. "
                "HANDOVER 8장 '5-1 폴백 후보 정량 비교' 참조.")

    # ------------------------------------------------------------------ params

    def _declare_params(self):
        self.declare_parameter("detections_topic", "/yolo_result")
        self.declare_parameter("depth_topic",
                               "/ascamera/camera_publisher/depth0/image_raw")
        self.declare_parameter("depth_info_topic",
                               "/ascamera/camera_publisher/depth0/camera_info")
        self.declare_parameter("color_info_topic", "/image_enhanced/camera_info")
        # ★ 발행할 x,y 의 기준. 메인이 역투영에 쓸 K와 **같아야** 합니다.
        self.declare_parameter("rgb0_info_topic",
                               "/ascamera/camera_publisher/rgb0/camera_info")
        self.declare_parameter("output_topic", "/fire/detections")
        self.declare_parameter("status_topic", "/fire/detections/status")

        self.declare_parameter("queue_size", 30)      # YOLO 지연 x Hz 를 덮을 것
        self.declare_parameter("slop_sec", 0.05)
        # ★ QoS 큐 깊이. sensor_data 기본값 5는 640x480 16UC1(614KB)에 얕아
        #   파이썬 노드가 절반 넘게 흘립니다 (실측 8.3Hz vs 13.2Hz, HANDOVER 5-2e).
        self.declare_parameter("qos_depth", 30)

        # 거리 샘플링 (depth.sample_distance_detail)
        # ★ 기본이 "center"가 아니라 "bottom"입니다 (2026-08-24 실측 반영).
        #   성냥불 위에서 뎁스가 안 나오는 것을 실기에서 확인했습니다 — 지시서 5-1이
        #   경고한 상황이 실제로 일어납니다. 화염은 스스로 적외선을 내는 광원이라
        #   박스 중앙(=불꽃)은 구조적으로 무효입니다. "center"로 두면 화재 검출마다
        #   `no_valid_pixels`로 버려집니다.
        #   여기 "region"은 **매핑에 없는 클래스**의 기본값입니다 — 클래스별 값은
        #   아래 region_by_class 를 보세요.
        self.declare_parameter("region", "bottom")
        self.declare_parameter("method", "median")
        self.declare_parameter("central", 0.5)

        # --- `below` 띠의 위치·두께 (박스 높이의 배수) ---
        # ★ 2026-08-26 실기 측정으로 정한 값입니다. 화염 박스 19px, 실제 화염
        #   1.5~2cm, 촛대를 받친 종이컵은 화염 아래 약 9cm.
        #     9cm / 1.75cm ≈ 5.1배  <- 눈대중 "5~6배"와 일치
        #   박스 바로 아래(band_offset=0)는 아직 화염 언저리라 배경(벽)이 잡혔습니다.
        #
        #   3.5 ~ 3.5+3.0 = 화염 아래 6.1~11.4cm (두께 5.3cm)를 덮습니다.
        #   9cm를 가운데 두고 위아래로 여유를 둔 건, 촛불이 타들어가면서
        #   화염-종이컵 거리가 **계속 변하기** 때문입니다. 얇게 잡으면 어느
        #   시점에 컵을 벗어납니다.
        #
        # ★ 배수로 잡는 이유: 9cm의 픽셀 크기도 박스 높이도 똑같이 1/Z 이라
        #   **비율이 거리와 무관**합니다. 거리를 몰라도 되고 f_y도 안 씁니다.
        # ★ band_offset > 0 이면 method가 자동으로 max -> median 이 됩니다.
        #   띠가 컵 몸통이라 가장 먼 값은 컵 뒤 배경입니다 (detection3d.method_for).
        self.declare_parameter("band_offset", 3.5)
        self.declare_parameter("band_ratio", 3.0)

        self.declare_parameter("ring_margin", 0.25)
        self.declare_parameter("min_valid_ratio", 0.2)
        self.declare_parameter("min_valid_px", 1)
        self.declare_parameter("z_min", 0.2)          # HP60C 스펙
        self.declare_parameter("z_max", 4.0)
        self.declare_parameter("max_spread_m", 0.0)   # <=0 이면 꺼짐

        # ★★ 클래스마다 재는 위치가 다릅니다 (2026-08-26 실기 실측).
        #
        #   fire:below    박스 **대부분이 불꽃**이라 박스 안에 대상 표면이 없습니다.
        #                 bottom으로 재면 불꽃 사이로 보이는 **배경(벽)**이 잡혀
        #                 +0.33~+0.60m 멀게 나갔습니다 — 그것도 유효비율 0.98에
        #                 사유 `ok`로요. below는 박스 바깥이라 불꽃 영향을 안 받습니다.
        #   person:bottom below로 쟀더니 **예상보다 가깝게** 나왔습니다. 사람 앞쪽
        #                 바닥을 재기 때문입니다. 사람은 박스 안 뎁스가 멀쩡하므로
        #                 굳이 주변을 잴 이유가 없습니다.
        #
        # 형식은 "클래스:영역" 쉼표 구분. 매핑에 없는 클래스는 위 `region`을 씁니다.
        # ★ 클래스 이름은 **학습 라벨과 정확히** 같아야 합니다 — `Fire`처럼 대소문자가
        #   다르면 조용히 안 걸리고 기본값(bottom)이 쓰입니다.
        # ★ method는 자동으로 따라갑니다 (below→max). region만 바꾸면 median이
        #   걸려 -0.326m가 나갑니다 — detection3d.SamplingParams.region_for 참고.
        self.declare_parameter("region_by_class", "fire:below,person:bottom")

        # ★ 폴백 기본 꺼짐. 켜기 전에 HANDOVER 8장 편향 표를 읽으세요.
        self.declare_parameter("fallback_regions", "")   # 예: "bottom,below,ring"
        # "" = 영역별 권장값 (below→max, bottom/ring→median). 아래 참조:
        #   detection3d.method_for
        self.declare_parameter("fallback_method", "")
        # 폴백 값을 실어 보낼지. 보낼 때는 `depth_status`가 폴백임을 밝힙니다.
        # False면 폴백 거리를 버리고 `unknown`으로 내보냅니다.
        self.declare_parameter("publish_fallback", True)
        self.declare_parameter("min_score", 0.0)
        # ★ 검출 0개 프레임은 안 냅니다. 살아있다는 말은 하트비트가 합니다.
        self.declare_parameter("publish_empty", False)
        self.declare_parameter("heartbeat_period_sec", 1.0)
        # 마지막 처리 프레임이 이보다 오래되면 `stalled`. 15Hz에 1초면 15프레임.
        self.declare_parameter("stall_after_sec", 1.0)

        # TF
        self.declare_parameter("stamp_history", 90)     # 15Hz 기준 약 6초
        self.declare_parameter("stamp_window", 30)
        self.declare_parameter("stamp_min_match", 0.5)
        self.declare_parameter("stats_period_sec", 5.0)

    def on_param_update(self, params):
        """bag 재생 중 파라미터를 돌려보기 위한 것. 5-1 폴백 실측에 씁니다."""
        for prm in params:
            n, v = prm.name, prm.value
            try:
                if n == "fallback_regions":
                    self.params.fallback_regions = tuple(
                        s for s in str(v).split(",") if s.strip())
                elif n == "region_by_class":
                    # 오타면 ValueError -> 아래에서 거부됩니다. 잘못된 매핑을
                    # 받아들이면 그 클래스만 조용히 다른 곳을 잽니다.
                    self.params.region_by_class = parse_region_by_class(str(v))
                elif n == "max_spread_m":
                    self.params.max_spread_m = None if float(v) <= 0 else float(v)
                elif n == "fallback_method":
                    self.params.fallback_method = str(v).strip() or None
                elif hasattr(self.params, n):
                    setattr(self.params, n, type(getattr(self.params, n))(v))
                elif n == "publish_fallback":
                    self.publish_fallback = bool(v)
            except (TypeError, ValueError) as e:  # noqa: PERF203
                return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    # -------------------------------------------------------------- camera_info

    def on_color_info(self, msg: CameraInfo):
        if self.k_color is None and not principal_point_sanity(msg.k, msg.width, msg.height):
            # 축소본에 원본 K를 쓰는 사고를 막는 유일한 방어선입니다.
            self.get_logger().error(
                f"컬러 K가 해상도({msg.width}x{msg.height})와 안 맞습니다 — "
                "전처리의 camera_info 재발행을 확인하세요. 거리가 배율만큼 틀립니다.")
        self.k_color = list(msg.k)
        self._color_size = (msg.width, msg.height)

    def on_depth_info(self, msg: CameraInfo):
        if self.k_depth is None:
            if not principal_point_sanity(msg.k, msg.width, msg.height):
                self.get_logger().error(
                    f"뎁스 K가 해상도({msg.width}x{msg.height})와 안 맞습니다.")
            if msg.distortion_model and msg.distortion_model != "plumb_bob":
                # 어안이면 backproject 공식이 성립하지 않습니다 (지시서 3-3).
                self.get_logger().error(
                    f"distortion_model={msg.distortion_model!r} — backproject는 "
                    "plumb_bob 전제입니다. cv2.undistortPoints가 먼저 필요합니다.")
        self.k_depth = list(msg.k)
        self._depth_size = (msg.width, msg.height)
        # camera_info 는 뎁스 이미지와 **같은 stamp**로 나옵니다(드라이버가 한
        # 콜백에서 발행). 작은 메시지라 stamp 수집에 쓰기 좋습니다.
        self.stamp_monitor.note_camera_stamp((msg.header.stamp.sec,
                                              msg.header.stamp.nanosec))

    def on_rgb0_info(self, msg: CameraInfo):
        """★ 발행할 픽셀 좌표계. 메인이 이 K로 역투영합니다.

        `frame_size`도 여기서 나옵니다 — 메인이 "이 x,y가 어느 해상도 기준인지"를
        알 방법이 이것뿐입니다.
        """
        if self.k_rgb0 is None and not principal_point_sanity(msg.k, msg.width,
                                                              msg.height):
            self.get_logger().error(
                f"rgb0 K가 해상도({msg.width}x{msg.height})와 안 맞습니다 — "
                "메인의 역투영이 통째로 틀어집니다.")
        self.k_rgb0 = list(msg.k)
        self.rgb0_size = (int(msg.width), int(msg.height))

    # ------------------------------------------------------------------- main

    def on_synced(self, det_msg: Detection2DArray, depth_msg: Image):
        self._n_in += 1
        self._n_frames_total += 1
        t0 = time.perf_counter()

        if self.k_color is None or self.k_depth is None or self.k_rgb0 is None:
            # ★ K 없이 추정값으로 발행하지 않습니다. 좌표가 조용히 틀립니다.
            self._note("waiting_camera_info")
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge 뎁스 변환 실패: {e}")
            return

        if not self._depth_checked:
            self._depth_checked = True
            if not depth_unit_sanity(to_meters(depth, encoding=depth_msg.encoding)):
                self.get_logger().error(
                    "뎁스 값의 중앙값이 실내 범위를 벗어납니다 — 단위(mm/m) 해석이 "
                    f"틀렸을 수 있습니다 (encoding={depth_msg.encoding}).")

        # 처리한 프레임 시각. 하트비트의 age가 여기서 나옵니다 — 검출이 없어도
        # 갱신해야 "불이 없는 정상"과 "멈춤"이 구분됩니다.
        self._last_frame = (int(det_msg.header.stamp.sec),
                            int(det_msg.header.stamp.nanosec))

        boxes = []
        for det in det_msg.detections:
            cid, score = ("", 0.0)
            if det.results:
                cid, score = hypothesis(det.results[0])
            boxes.append((box_from_bbox(det.bbox), cid, score))

        result = convert_frame_pixels(
            boxes, depth, self.k_color, self.k_depth, self.k_rgb0,
            self.params, encoding=depth_msg.encoding)

        entries = [self._filtered(e) for e in result.entries]
        self._n_unknown += sum(1 for e in entries if e["depth"] is None)
        # 대상이 아니라 주변을 잰 것만 셉니다. `fallback_bottom`은 박스 안이라
        # 제외 — 기본값이라 세면 매 프레임 전건이 잡혀 로그가 무의미해집니다.
        self._n_fallback += sum(1 for e in entries
                                if is_surrogate(e["depth_status"]))

        if entries or self.publish_empty:
            msg = String()
            # ★★ 원본 이미지의 stamp. 발행 시각(now())이 아닙니다 (HANDOVER 4-8).
            msg.data = to_json(build_payload(*self._last_frame,
                                             self.rgb0_size, entries))
            self.pub.publish(msg)
            self._n_out += len(entries)
            self._n_pub_total += 1

        for reason, n in result.reason_counts().items():
            self._reasons[reason] = self._reasons.get(reason, 0) + n
        self._t_proc.append((time.perf_counter() - t0) * 1000.0)
        self._report_if_due()

    def _filtered(self, entry):
        """`publish_fallback=False`면 폴백 거리를 버리고 불명으로 표시합니다.

        ★ 거리만 지우고 **검출은 남깁니다.** 빼버리면 메인은 "불이 보였다"는
        사실조차 모릅니다.

        ★ 여기서 "폴백"은 **주변을 대신 잰 것**(`below`/`ring`)만입니다.
        `fallback_bottom`은 이름과 달리 박스 **안**이라 대상 자체를 읽으므로
        거르지 않습니다 — 기본 region이 `bottom`이라, 접두사로 걸렀다면
        `publish_fallback=false`가 모든 거리를 지워버립니다.
        """
        if self.publish_fallback or not is_surrogate(entry["depth_status"]):
            return entry
        out = dict(entry)
        out["depth"] = None
        out["depth_status"] = STATUS_UNKNOWN
        return out

    def _count_depth(self, _msg):
        self._n_depth_seen += 1
        self._n_depth_total += 1

    def on_detection_seen(self, msg: Detection2DArray):
        """동기화 전 검출 도착 — stamp 감시 + 동기화 실패 진단용."""
        self._n_det_seen += 1
        self._n_det_total += 1
        if self.stamp_monitor.check_detection((msg.header.stamp.sec,
                                               msg.header.stamp.nanosec)):
            self.get_logger().error(self.stamp_monitor.message())

    def _heartbeat(self):
        """★ 살아있음만 말하는 하트비트는 거짓말을 합니다.

        동기화가 영원히 안 맞아도 타이머는 정상 동작하므로, 단순 펄스는 계속
        "정상"을 주장합니다. 그래서 **마지막으로 처리한 프레임의 stamp와 경과
        시간**을 실어 받는 쪽이 직접 판정할 수 있게 합니다.
        """
        now = self.get_clock().now().to_msg()
        now_pair = (int(now.sec), int(now.nanosec))
        age = stamp_age_sec(now_pair, self._last_frame)
        state = heartbeat_state(
            age,
            camera_info_ready=None not in (self.k_color, self.k_depth, self.k_rgb0),
            inputs_seen=bool(self._n_det_total and self._n_depth_total),
            stall_after_sec=self.stall_after_sec)

        msg = String()
        msg.data = to_json(build_heartbeat(
            *now_pair, state, last_frame=self._last_frame, age_sec=age,
            counters={"published": self._n_pub_total,
                      "frames": self._n_frames_total,
                      "unknown_depth": self._n_unknown,
                      "fallback_depth": self._n_fallback,
                      "detections_in": self._n_det_total,
                      "depth_in": self._n_depth_total}))
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------ stats

    def _note(self, reason: str):
        self._reasons[reason] = self._reasons.get(reason, 0) + 1
        self._report_if_due()

    def _report_if_due(self):
        now = time.monotonic()
        if now - self._last_report < self.stats_period:
            return
        elapsed = now - self._last_report
        msg = (f"[태스크②] 검출 {self._n_det_seen / elapsed:4.1f}Hz "
               f"| 뎁스 {self._n_depth_seen / elapsed:4.1f}Hz "
               f"| 동기화 {self._n_in / elapsed:4.1f}Hz | 발행 {self._n_out}개 "
               f"(누적 불명 {self._n_unknown} / 폴백 {self._n_fallback})")
        if self._n_det_seen and self._n_in == 0:
            # 검출은 오는데 하나도 안 맞음 = stamp 가 뎁스와 전혀 다른 시간대
            msg += "  <-- ★ 검출이 오는데 동기화가 0건입니다. stamp 불일치(slop 초과)를 " \
                   "의심하세요"
        if self._t_proc:
            msg += f" | 처리 {np.mean(self._t_proc):5.2f}ms"
        if self._reasons:
            # ★ "왜 좌표가 안 나오는지"를 현장에서 알 수 없으면 5-1 대응을 못 고릅니다
            msg += " | 제외: " + ", ".join(f"{k}={v}" for k, v in
                                           sorted(self._reasons.items()))
        self.get_logger().info(msg)
        self._n_in = self._n_out = self._n_det_seen = self._n_depth_seen = 0
        self._reasons.clear()
        self._t_proc.clear()
        self._last_report = now


def _sensor_qos(depth: int) -> QoSProfile:
    """`qos_profile_sensor_data`와 **호환되면서** 큐만 깊게.

    ★ `reliability`는 절대 바꾸지 마세요. 드라이버가 BEST_EFFORT로 발행하므로
    RELIABLE로 구독하면 매칭이 안 돼 **콜백이 영원히 안 불립니다** — 에러 없이
    노드가 침묵합니다. 큐 깊이는 매칭에 영향을 주지 않아 안전하게 늘립니다
    (기본 5로는 614KB 뎁스를 절반 넘게 흘립니다 — HANDOVER 5-2e).
    """
    qos = QoSProfile(depth=depth)
    qos.reliability = qos_profile_sensor_data.reliability
    qos.durability = qos_profile_sensor_data.durability
    qos.history = qos_profile_sensor_data.history
    return qos


def main(args=None):
    rclpy.init(args=args)
    node = Detection3DNode()
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
