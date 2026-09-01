#!/usr/bin/env python3
"""
태스크 ① RGB 전처리 노드 — 연기/저조도 극복

  /camera/color/image_raw  --(디헤이즈 -> CLAHE)-->  /image_enhanced

로드맵의 3단계 구현을 **하나의 노드 + mode 파라미터**로 합쳤습니다.
단계별로 새 파일을 만드는 대신 mode만 바꿔가며 올라가면 되고,
나중에 3조건 비교 실험도 같은 바이너리로 돌아갑니다.

  mode: passthrough  -> 1단계 뼈대 (무처리 통과, QoS/헤더만 검증)
        clahe        -> 2단계
        dehaze       -> 디헤이즈 단독 (기여도 분리용)
        full         -> 3단계 (디헤이즈 -> CLAHE)

mode는 런타임 변경 가능:
  ros2 param set /rgb_preprocess_node mode clahe

주의: 이 노드가 잡아주는 "조용한 실패" 3가지
  1) QoS   — 입출력 모두 qos_profile_sensor_data (정수 10 쓰면 콜백이 영원히 안 불림)
  2) 헤더  — cv2_to_imgmsg 결과에 입력 헤더를 그대로 복사 (없으면 태스크②의 TF가 죽음)
  3) 해상도 — process_width로 축소하면 K도 같이 바뀌어야 함 -> camera_info 재발행
"""

from __future__ import annotations

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from image_pipeline.dehaze import ClaheEnhancer, DarkChannelDehazer
from image_pipeline.intrinsics import fit_size, scale_k, scale_p
from image_pipeline.autotune import AdaptiveParams
from image_pipeline.pipeline import MODES, Pipeline


class PreprocessNode(Node):
    def __init__(self):
        super().__init__("rgb_preprocess_node")

        self._declare_params()
        p = self.get_parameter

        # ★ OpenCV 스레드 수. **다른 무엇보다 먼저** 잡아야 이후 cv2 호출에 적용됩니다.
        #   기본값(=코어 수)이면 지연시간은 줄지만 총 CPU가 오히려 늘어납니다.
        #   RPi5 4코어, 640x480, mode=full, 현재 코드 실측 (프레임당):
        #       threads=4  wall 14.2ms / CPU 34.1ms (코어 2.40개)
        #       threads=2  wall 15.8ms / CPU 24.5ms
        #       threads=1  wall 19.3ms / CPU 19.3ms  <- CPU -43%
        #   CLAHE가 특히 심해서 wall 3.3ms 를 위해 CPU 13.2ms(코어 4.00)를 씁니다.
        #   카메라가 15fps(ascamera.launch.py)라 예산이 66ms고, threads=1 이어도
        #   51fps 여력이 남습니다. nav2/SLAM/YOLO와 코어를 나눠 쓰므로 지연보다
        #   CPU 점유를 줄이는 쪽이 이득입니다.
        #   ⚠ 프로세스 전역 설정입니다. 0이면 건드리지 않음(yolo_node와 같은 규약).
        threads = int(p("threads").value)
        if threads > 0:
            cv2.setNumThreads(threads)

        mode = p("mode").value
        if mode not in MODES:
            self.get_logger().warn(f"알 수 없는 mode '{mode}' -> 'full'로 대체")
            mode = "full"

        self.process_width = int(p("process_width").value)

        # 처리 로직은 Pipeline(ROS 비의존)에 있습니다. 노드는 배선만 담당.
        # -> tests/ 에서 rclpy 없이 같은 코드를 검증할 수 있습니다.
        self.pipe = Pipeline(
            mode=mode,
            gamma=float(p("gamma").value),
            lowlight=bool(p("lowlight_dehaze").value),
            clahe=ClaheEnhancer(
                float(p("clahe_clip_limit").value),
                tuple(int(v) for v in p("clahe_tile_grid").value),
            ),
            dehazer=DarkChannelDehazer(
                omega=float(p("dehaze_omega").value),
                t0=float(p("dehaze_t0").value),
                patch=int(p("dehaze_patch").value),
                scale=float(p("dehaze_scale").value),
                use_guided=bool(p("dehaze_use_guided").value),
                guided_radius=int(p("dehaze_guided_radius").value),
                guided_eps=float(p("dehaze_guided_eps").value),
                a_max=float(p("dehaze_a_max").value),
                sky_ratio=float(p("dehaze_sky_ratio").value),
                a_smoothing=float(p("dehaze_a_smoothing").value),
            ),
        )

        self.bridge = CvBridge()
        self.scale_x = 1.0
        self.scale_y = 1.0

        in_topic = p("input_topic").value
        out_topic = p("output_topic").value

        # ★ 발행/구독 모두 sensor QoS. 한쪽만 맞춰도 소용 없습니다.
        self.pub = self.create_publisher(Image, out_topic, qos_profile_sensor_data)
        self.sub = self.create_subscription(
            Image, in_topic, self.on_image, qos_profile_sensor_data
        )

        # camera_info 스케일 재발행 (태스크②가 축소된 좌표계를 쓰게 될 경우 필수)
        self.info_pub = None
        self.info_sub = None
        if bool(p("publish_camera_info").value):
            info_in = p("camera_info_topic").value
            info_out = out_topic.rsplit("/", 1)[0] + "/camera_info" \
                if "/" in out_topic.strip("/") else out_topic + "/camera_info"
            info_out = p("output_camera_info_topic").value or info_out
            self.info_pub = self.create_publisher(CameraInfo, info_out, qos_profile_sensor_data)
            self.info_sub = self.create_subscription(
                CameraInfo, info_in, self.on_camera_info, qos_profile_sensor_data
            )
            self.get_logger().info(f"camera_info: {info_in} -> {info_out} (K 스케일 보정)")

        # --- 자동 적응 ---
        # 임무 중 연기 농도가 변하므로 고정값은 어느 한쪽에서 반드시 틀립니다.
        # 다만 조건 비교 실험 중에는 꺼야 해석이 쉬우므로 기본값은 False입니다.
        self.autotuner = None
        if bool(p("auto_tune").value):
            baseline = float(p("auto_tune_haze_baseline").value)
            self.autotuner = AdaptiveParams(
                smoothing=float(p("auto_tune_smoothing").value),
                update_every=int(p("auto_tune_every").value),
                haze_baseline=None if baseline < 0 else baseline,
            )
            self.get_logger().info(
                "auto_tune 켜짐 — omega/t0/clipLimit이 장면에 따라 자동 조정됩니다. "
                "(조건 비교 실험 시에는 끄세요)"
            )

        # --- 성능 계측 ---
        self.stats_period = float(p("stats_period_sec").value)
        self._t_total: list[float] = []
        self._t_dehaze: list[float] = []
        self._t_clahe: list[float] = []
        self._n_in = 0
        self._last_report = time.monotonic()

        self.add_on_set_parameters_callback(self.on_param_update)

        self.get_logger().info(
            f"[태스크①] {in_topic} -> {out_topic} | mode={self.pipe.mode} "
            f"| process_width={self.process_width or '원본'} "
            # 실제로 몇 스레드로 도는지 남깁니다. 파라미터를 줬는데 안 먹은 경우
            # (오타로 declare 가 안 됐다든지) 로그만 보고 알 수 있어야 합니다.
            f"| cv2 threads={cv2.getNumThreads()}"
        )

    # ------------------------------------------------------------------ params

    def _declare_params(self):
        # ★ 실제 드라이버(ascamera, Angstrong HP60C)의 토픽입니다.
        #   RGB는 "/image" 이고 "/image_raw"가 아닙니다 — 뎁스만 "_raw"가 붙습니다.
        #   근거: third_party_ws/src/ascamera/src/CameraPublisher.cpp:197-200
        #        + src/peripherals/launch/include/ascamera.launch.py (namespace "ascamera")
        self.declare_parameter("input_topic", "/ascamera/camera_publisher/rgb0/image")
        self.declare_parameter("output_topic", "/image_enhanced")
        self.declare_parameter("camera_info_topic",
                               "/ascamera/camera_publisher/rgb0/camera_info")
        self.declare_parameter("output_camera_info_topic", "/image_enhanced/camera_info")
        self.declare_parameter("publish_camera_info", True)

        self.declare_parameter("mode", "full")
        # ⚠ 현재 카메라 설정(RGB 640x480)에서는 **무동작**입니다.
        #   fit_size는 target_w >= src_w면 원본을 그대로 둡니다(확대 안 함).
        #   성냥불 실측 후 RGB를 1080p로 올리면 그때부터 실제로 축소가 걸립니다.
        self.declare_parameter("process_width", 640)   # 0 = 원본 유지
        self.declare_parameter("gamma", 1.0)           # <1 이면 밝아짐
        self.declare_parameter("lowlight_dehaze", False)

        self.declare_parameter("clahe_clip_limit", 2.0)
        self.declare_parameter("clahe_tile_grid", [8, 8])

        self.declare_parameter("dehaze_omega", 0.95)
        self.declare_parameter("dehaze_t0", 0.1)
        self.declare_parameter("dehaze_patch", 15)
        self.declare_parameter("dehaze_scale", 0.25)
        self.declare_parameter("dehaze_use_guided", True)
        self.declare_parameter("dehaze_guided_radius", 8)
        self.declare_parameter("dehaze_guided_eps", 0.001)
        self.declare_parameter("dehaze_a_max", 0.92)
        self.declare_parameter("dehaze_sky_ratio", 1.0)
        self.declare_parameter("dehaze_a_smoothing", 0.0)

        # 자동 파라미터 적응 (autotune.AdaptiveParams)
        self.declare_parameter("auto_tune", False)
        self.declare_parameter("auto_tune_smoothing", 0.9)
        self.declare_parameter("auto_tune_every", 10)
        self.declare_parameter("auto_tune_haze_baseline", -1.0)  # 음수 = 자동 학습

        # OpenCV 스레드 수. 0 = 건드리지 않음(= 코어 수). __init__ 의 주석 참고.
        self.declare_parameter("threads", 0)

        self.declare_parameter("stats_period_sec", 5.0)

    def on_param_update(self, params):
        """재빌드/재실행 없이 파라미터 튜닝. bag 재생 중에 값 돌려보기 좋습니다."""
        for prm in params:
            n, v = prm.name, prm.value
            if n == "mode":
                if v not in MODES:
                    return SetParametersResult(
                        successful=False, reason=f"mode는 {MODES} 중 하나"
                    )
                self.pipe.set_mode(v)
                self.get_logger().info(f"mode -> {v}")
            elif n == "threads":
                # 여기서 안 받으면 `ros2 param set ... threads 4` 가 **성공만 하고
                # 아무 일도 안 합니다.** 로봇에서 스레드 수를 A/B 비교할 때
                # 재실행 없이 돌려볼 수 있어야 하므로 런타임 반영합니다.
                # (0 = 건드리지 않음이므로 되돌리려면 명시적으로 값을 줘야 합니다)
                if int(v) > 0:
                    cv2.setNumThreads(int(v))
                self.get_logger().info(
                    f"threads -> {v} (실제 cv2 스레드 {cv2.getNumThreads()})")
            elif n == "process_width":
                self.process_width = int(v)
            elif n == "gamma":
                self.pipe.gamma = float(v)
            elif n == "lowlight_dehaze":
                self.pipe.lowlight = bool(v)
            elif n == "clahe_clip_limit":
                self.pipe.clahe.update(float(v), self.pipe.clahe.tile_grid)
            elif n == "clahe_tile_grid":
                self.pipe.clahe.update(self.pipe.clahe.clip_limit,
                                       tuple(int(x) for x in v))
            elif n.startswith("dehaze_"):
                attr = {
                    "dehaze_omega": "omega",
                    "dehaze_t0": "t0",
                    "dehaze_patch": "patch",
                    "dehaze_scale": "scale",
                    "dehaze_use_guided": "use_guided",
                    "dehaze_guided_radius": "guided_radius",
                    "dehaze_guided_eps": "guided_eps",
                    "dehaze_a_max": "a_max",
                    "dehaze_sky_ratio": "sky_ratio",
                    "dehaze_a_smoothing": "a_smoothing",
                }.get(n)
                if attr:
                    setattr(self.pipe.dehazer, attr, v)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ info

    def on_camera_info(self, msg: CameraInfo):
        """process_width로 축소한 만큼 K를 함께 줄여 재발행.

        이걸 안 하면 태스크②가 원본 K로 축소 이미지 좌표를 역투영해
        **에러 없이 거리가 틀립니다.** 정확히 로드맵이 경고한 유형의 사고.
        """
        if self.info_pub is None:
            return
        # 실제 이미지 콜백에서 쓴 것과 **같은 함수**로 크기·배율을 구합니다.
        # (여기서만 따로 계산하면 반올림 차이로 cx가 어긋납니다.)
        _, _, sx, sy = fit_size(msg.width, msg.height, self.process_width)
        new_w = int(round(msg.width * sx))
        new_h = int(round(msg.height * sy))

        out = CameraInfo()
        out.header = msg.header
        out.width = new_w
        out.height = new_h
        out.distortion_model = msg.distortion_model
        out.d = list(msg.d)  # 왜곡 계수는 정규화 좌표계 기준이라 스케일 불변
        out.k = scale_k(msg.k, sx, sy)
        out.p = scale_p(msg.p, sx, sy)
        out.r = list(msg.r)
        self.info_pub.publish(out)

    # ------------------------------------------------------------------ main

    def on_image(self, msg: Image):
        t_start = time.perf_counter()
        self._n_in += 1

        try:
            # ★ encoding 명시. 뎁스가 아닌 컬러이므로 bgr8.
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge 변환 실패: {e}")
            return

        # --- 다운스케일 (성능) ---
        h, w = img.shape[:2]
        new_w, new_h, self.scale_x, self.scale_y = fit_size(w, h, self.process_width)
        if (new_w, new_h) != (w, h):
            # 축소는 INTER_AREA. INTER_LINEAR로 줄이면 에일리어싱이 생겨
            # 작은 불씨가 사라질 수 있습니다.
            img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

        # 자동 적응이 켜져 있으면 처리 전에 파라미터를 갱신합니다.
        if self.autotuner is not None:
            self.autotuner.update(img, self.pipe)

        # ★ 처리 본체. 오프라인 도구·테스트와 완전히 같은 코드 경로입니다.
        img = self.pipe.process(img)

        try:
            out = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"cv_bridge 역변환 실패: {e}")
            return

        # ★★ 헤더 복사 — 이 한 줄이 없으면 stamp=0, frame_id="" 로 나가서
        #     태스크②의 TF 조회가 통째로 실패합니다.
        out.header = msg.header

        self.pub.publish(out)

        total = (time.perf_counter() - t_start) * 1000.0
        self._t_total.append(total)
        self._t_dehaze.append(self.pipe.timings["dehaze"])
        self._t_clahe.append(self.pipe.timings["clahe"])
        self._report_if_due()

    # ------------------------------------------------------------------ stats

    def _report_if_due(self):
        now = time.monotonic()
        if now - self._last_report < self.stats_period or not self._t_total:
            return
        elapsed = now - self._last_report

        arr = np.array(self._t_total)
        mean = arr.mean()
        p95 = float(np.percentile(arr, 95))
        in_hz = self._n_in / elapsed
        budget = 1000.0 / in_hz if in_hz > 0 else float("inf")

        msg = (
            f"[perf] in {in_hz:4.1f}Hz | 처리 평균 {mean:6.2f}ms (p95 {p95:6.2f}ms) "
            f"| 이론 최대 {1000.0 / max(mean, 1e-6):5.1f}fps"
        )
        if self.pipe.mode in ("dehaze", "full"):
            msg += f" | 디헤이즈 {np.mean(self._t_dehaze):5.2f}ms"
        if self.pipe.mode in ("clahe", "full"):
            msg += f" | CLAHE {np.mean(self._t_clahe):5.2f}ms"

        if mean > budget:
            self.get_logger().warn(
                msg + f"  <-- 입력 주기 {budget:.1f}ms 초과. 병목입니다. "
                "process_width를 낮추거나 dehaze_scale을 줄이세요."
            )
        else:
            self.get_logger().info(msg)

        self._t_total.clear()
        self._t_dehaze.clear()
        self._t_clahe.clear()
        self._n_in = 0
        self._last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = PreprocessNode()
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
