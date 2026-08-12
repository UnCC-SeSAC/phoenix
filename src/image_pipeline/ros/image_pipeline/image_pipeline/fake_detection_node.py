#!/usr/bin/env python3
"""
가짜 검출 + 가짜 뎁스 노드 — YOLO·로봇 없이 태스크②를 끝까지 돌려보기 위한 도구.

`fake_camera_node.py`와 같은 발상입니다. 한 노드가 YOLO와 뎁스 카메라를 동시에
흉내 내는 이유는 **정답을 아는 장면**을 만들기 위해서입니다. 박스와 뎁스가 따로
놀면 "좌표가 맞는지"를 확인할 기준이 없습니다.

  /yolo_result                 (vision_msgs/Detection2DArray)  박스는 컬러 좌표계
  /camera/depth/image_raw      (16UC1, mm)  640x480
  /camera/depth/camera_info
  /image_enhanced/camera_info  (전처리가 축소해 발행하는 것과 같은 K)  640x360

  ros2 run image_pipeline fake_detection_node --ros-args -p distance_m:=3.2

★ 일부러 재현하는 함정 3가지 — 다 끄고 만들면 태스크② 노드가 로봇에서 처음
  깨집니다.

  1. **컬러와 뎁스의 해상도도 화각도 다릅니다** — 실제 하드웨어
     (Angstrong Nuwa-HP60C)가 컬러 1920x1080(16:9), 뎁스 640x480(4:3)입니다.
     박스는 컬러 좌표계로 나가므로 받는 쪽이 **`project_box`로 K를 거쳐**
     옮겨야 합니다. 해상도 배율(`scale_box`)로 옮기면 화면 중앙만 맞고
     위아래로 갈수록 어긋납니다(3.2m에서 세로 17cm). 여기서 해상도나
     가로세로비를 맞춰버리면 그 버그가 안 잡힙니다.
  2. **`qos_profile_sensor_data`로 발행합니다.** 구독 쪽에서 정수 10을 쓰면
     콜백이 영원히 안 불리는 걸 여기서 미리 겪습니다.
  3. **한 틱의 모든 메시지가 같은 stamp를 씁니다.** 태스크②는 이 stamp를
     그대로 출력에 실어야 합니다 (HANDOVER 4-8). 여기서 stamp가 셋 다 다르면
     동기화 버그와 stamp 전파 버그를 구분할 수 없습니다.

⚠ **거리 기본값 3.2m는 이 카메라 스펙(0.2~4m)의 80% 지점입니다.** 스테레오는
거리 제곱에 비례해 오차가 커지므로 여기가 가장 불리한 구간이고, 4m를 넘으면
아예 값이 없습니다. `distance_m`을 2m 근처로도 돌려보세요 (`HANDOVER.md` 8장 P0).

`flame_hole:=true`로 켜면 박스 영역의 뎁스가 통째로 0이 됩니다. RealSense가
화염(적외선 광원) 위에서 거리를 못 재는 상황(지시서 5-1)의 재현이고, 이때
태스크②는 좌표를 지어내지 말고 **빼야** 합니다.

계산은 전부 `image_pipeline/depth.py`에 있습니다. 이 파일은 배선만 합니다.
"""

from __future__ import annotations

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from image_pipeline.depth import box_center, dummy_scene, scale_box
from image_pipeline.detection_msgs import set_bbox_center, set_hypothesis
from image_pipeline.intrinsics import principal_point_sanity, scale_k


class FakeDetection(Node):
    def __init__(self):
        super().__init__("fake_detection_node")

        self._declare_params()
        p = self.get_parameter

        self.class_id = str(p("class_id").value)
        self.score = float(p("score").value)
        self.color_frame = str(p("color_frame_id").value)
        self.depth_frame = str(p("depth_frame_id").value)

        # ★ 장면 조립은 depth.dummy_scene()에 있습니다 (노드는 배선만).
        #   테스트가 **같은 함수**로 같은 장면을 만들어 검산하므로, 더미가 틀렸는지
        #   태스크②가 틀렸는지를 구분할 수 있습니다.
        cx = float(p("box_center_x").value)
        cy = float(p("box_center_y").value)
        color_w, color_h = int(p("color_width").value), int(p("color_height").value)
        self.scene = dummy_scene(
            color_size=(color_w, color_h),
            depth_size=(int(p("depth_width").value), int(p("depth_height").value)),
            box_size=(float(p("box_width").value), float(p("box_height").value)),
            box_center_xy=(cx if cx >= 0 else color_w / 2.0,
                           cy if cy >= 0 else color_h / 2.0),
            distance_m=float(p("distance_m").value),
            background_m=float(p("background_m").value),
            color_hfov_deg=float(p("color_hfov_deg").value),
            depth_hfov_deg=float(p("depth_hfov_deg").value),
            flame_hole=bool(p("flame_hole").value),
            noise_m=float(p("noise_m").value),
            floor_height_m=(float(p("floor_height_m").value)
                            if float(p("floor_height_m").value) > 0 else None),
            box_on_floor=float(p("floor_height_m").value) > 0,
        )

        if not principal_point_sanity(self.scene.k_color, color_w, color_h):
            self.get_logger().error("생성한 컬러 K가 해상도와 안 맞습니다 — 버그입니다")

        # ★ 전처리(태스크①)가 `process_width`로 축소하면 박스도 축소본 좌표로
        #   나옵니다. 태스크②는 그걸 **원본 좌표로 되돌려** 발행해야 하고
        #   (메인이 rgb0/camera_info로 역투영하므로), 되돌리기를 빠뜨려도 화면
        #   중앙 근처에서는 티가 안 납니다. 그래서 여기서 일부러 축소본을 만듭니다.
        #   현재 process_width=640은 원본과 같아 축소가 안 걸리므로, 이 파라미터가
        #   없으면 그 버그가 **로봇에서 처음** 나타납니다.
        enh_w = int(p("enhanced_width").value) or color_w
        self.enh_scale = enh_w / float(color_w)
        self.enh_size = (enh_w, int(round(color_h * self.enh_scale)))
        self.k_enhanced = scale_k(self.scene.k_color, self.enh_scale, self.enh_scale)
        # 발행할 박스는 **축소본 좌표**. 실제 계통에서 YOLO가 내는 것과 같습니다.
        self.box_enhanced = scale_box(self.scene.box_color,
                                      self.enh_scale, self.enh_scale)

        self.bridge = CvBridge()
        # ★ 배열이 아니라 **Image 메시지**를 캐시합니다.
        #   cv2_to_imgmsg 는 0.61MB 를 매번 새로 담느라 파이썬에서 느립니다.
        #   배열만 캐시했을 때 뎁스가 10.6Hz 밖에 안 나가서, 태스크② 쪽
        #   동기화율이 낮은 것처럼 보였습니다(실제로는 더미가 병목).
        self._depth_msg = None if self.scene.noise_m else self._make_depth_msg()

        # --- 발행 ---
        self.det_pub = self.create_publisher(
            Detection2DArray, str(p("detections_topic").value), qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(
            Image, str(p("depth_topic").value), qos_profile_sensor_data)
        self.depth_info_pub = self.create_publisher(
            CameraInfo, str(p("depth_info_topic").value), qos_profile_sensor_data)
        self.color_info_pub = self.create_publisher(
            CameraInfo, str(p("color_info_topic").value), qos_profile_sensor_data)
        # ★ 원본 컬러 K. 메인이 역투영에 쓸 것과 같고, 태스크②가 발행할 x,y의
        #   기준입니다. 드라이버가 실제로 내는 토픽 이름을 씁니다.
        self.rgb0_info_pub = self.create_publisher(
            CameraInfo, str(p("rgb0_info_topic").value), qos_profile_sensor_data)

        self._log_ground_truth()

        fps = float(p("fps").value)
        self.timer = self.create_timer(1.0 / max(fps, 1e-3), self.tick)

    # ------------------------------------------------------------------ params

    def _declare_params(self):
        # 실제 YOLO 노드(yolov5_ros2)가 쓰는 이름과 맞춥니다. 더미가 다른
        # 이름으로 내면 태스크② 노드를 기본값으로 못 돌려봅니다.
        self.declare_parameter("detections_topic", "/yolo_result")
        # ★ 실제 드라이버(ascamera) 토픽. 소스에서 확인했습니다:
        #   CameraPublisher.cpp:193-196 + peripherals/launch/include/ascamera.launch.py
        self.declare_parameter("depth_topic", "/ascamera/camera_publisher/depth0/image_raw")
        self.declare_parameter("depth_info_topic",
                               "/ascamera/camera_publisher/depth0/camera_info")
        self.declare_parameter("color_info_topic", "/image_enhanced/camera_info")
        self.declare_parameter("rgb0_info_topic",
                               "/ascamera/camera_publisher/rgb0/camera_info")
        # 0 = 축소 없음. 320 등으로 주면 태스크②의 '원본 좌표 되돌리기'가
        # 실제로 검증됩니다 (안 되돌려도 화면 중앙은 맞아 보입니다).
        self.declare_parameter("enhanced_width", 0)

        # ★ 실제 launch 설정: rgb_width/height = depth_width/height = 640x480.
        #   (인수인계 문서의 "RGB 1080p"는 틀렸습니다 — launch가 640x480으로 엽니다.)
        #   따라서 지금은 가로세로비가 같고 project_box가 항등에 가깝습니다.
        #   성냥불 실측으로 RGB를 1080p(16:9)로 올리면 뎁스 4:3과 어긋나
        #   §3-1 함정이 되살아납니다 — 그래서 project_box를 계속 씁니다.
        self.declare_parameter("color_width", 640)
        self.declare_parameter("color_height", 480)
        self.declare_parameter("depth_width", 640)
        self.declare_parameter("depth_height", 480)
        # HP60C의 실제 화각은 미확인 — 로드맵의 60도 가정. 수령 후 camera_info로 교체.
        self.declare_parameter("color_hfov_deg", 60.0)
        self.declare_parameter("depth_hfov_deg", 60.0)

        self.declare_parameter("box_center_x", -1.0)     # 음수 = 화면 중앙
        self.declare_parameter("box_center_y", -1.0)
        self.declare_parameter("box_width", 80.0)        # 컬러 픽셀
        self.declare_parameter("box_height", 80.0)

        self.declare_parameter("distance_m", 3.2)        # 정답 거리 (스펙 상한 4m의 80%)
        self.declare_parameter("background_m", 3.8)      # 4m 밖은 측정이 안 됩니다
        self.declare_parameter("noise_m", 0.0)
        self.declare_parameter("flame_hole", False)      # 지시서 5-1 재현
        # >0 이면 배경이 평면 벽이 아니라 **바닥 + 벽**이 되고 불이 바닥에 놓입니다.
        # 5-1 폴백(bottom/below/ring)을 비교하려면 이 장면이어야 합니다.
        self.declare_parameter("floor_height_m", 0.0)    # 카메라의 바닥 위 높이

        self.declare_parameter("class_id", "fire")
        self.declare_parameter("score", 0.9)
        self.declare_parameter("fps", 15.0)   # 드라이버 launch 기본값
        # ★ 계약 위반을 일부러 재현합니다. 검출 stamp만 늦춰서 발행하면
        #   태스크②의 StampMonitor 가 실제로 무는지 확인할 수 있습니다.
        #   (파이프라인 중간이 stamp = now() 로 덮어쓴 상황과 같습니다)
        self.declare_parameter("break_stamp_sec", 0.0)

        self.declare_parameter("color_frame_id", "camera_color_optical_frame")
        self.declare_parameter("depth_frame_id", "camera_depth_optical_frame")
        # 정답 좌표 로그용. 실기에서는 URDF/tf2가 정답입니다 (여기 값은 참고용).
        self.declare_parameter("camera_offset_xyz", [0.1, 0.0, 0.35])

    # -------------------------------------------------------------- ground truth

    def _log_ground_truth(self):
        """이 설정에서 태스크② 노드가 발행해야 할 값을 시작 시 찍어둡니다.

        더미로 개발할 때 "맞는지"를 판정할 기준이 없으면 노드가 뭘 뱉든
        그럴듯해 보입니다. 그래서 정답을 노드가 직접 계산해 보여줍니다.
        """
        sc = self.scene
        u, v = box_center(sc.box_color)
        offset = [float(x) for x in self.get_parameter("camera_offset_xyz").value]
        base = sc.expected_base_link(offset)

        cw, ch = sc.color_size
        dw, dh = sc.depth_size
        same_aspect = abs(cw / ch - dw / dh) < 1e-6
        note = ("가로세로비가 같아 project_box가 사실상 항등입니다"
                if same_aspect else
                "★ 가로세로비가 다릅니다 — scale_box로 옮기면 위아래로 어긋납니다")
        self.get_logger().info(
            f"[더미] 컬러 {cw}x{ch} ({cw / ch:.2f}:1) / 뎁스 {dw}x{dh} ({dw / dh:.2f}:1) "
            f"— {note}")
        ue, ve = box_center(self.box_enhanced)
        self.get_logger().info(
            f"[더미] 발행하는 박스는 **축소본** {self.enh_size[0]}x{self.enh_size[1]} "
            f"좌표 center=({ue:.1f}, {ve:.1f}) | 거리 {sc.distance_m:.2f}m")

        if base is None:
            # 2026-08-10 계약: 좌표가 아니라 거리를 냅니다. 불명이면 null.
            self.get_logger().warn(
                "flame_hole=true — 박스 영역 뎁스가 전부 0입니다. 폴백이 없으면 "
                '태스크②는 "depth": null, "depth_status": "unknown" 을 내야 합니다 '
                "(0.0을 내면 메인이 로봇 발밑을 역투영합니다).")
        else:
            self.get_logger().info(
                f'[정답] {{"x": {u:.0f}, "y": {v:.0f}, "depth": {sc.distance_m:.3f}, '
                f'"depth_status": "ok"}} | frame_size=[{cw}, {ch}]')
            if self.enh_scale != 1.0:
                self.get_logger().info(
                    f"[정답] 축소본 좌표({ue:.0f}, {ve:.0f})를 그대로 발행하면 "
                    f"배율 {self.enh_scale:.2f}만큼 틀립니다 — 원본으로 되돌려야 합니다")
            self.get_logger().info(
                "[정답] 박스를 안 옮기거나 해상도 배율(scale_box)로 뎁스에 옮기면 배경 "
                f"{sc.background_m:.1f}m가 나옵니다 — 그게 지시서 3-1 함정입니다.")
            # 역투영은 메인 몫이지만, 우리 거리가 맞는지 되짚는 기준으로 남깁니다.
            opt = sc.expected_optical()
            self.get_logger().info(
                f"[참고] 메인이 이걸로 계산해야 할 값: optical=("
                f"{opt[0]:+.3f}, {opt[1]:+.3f}, {opt[2]:+.3f}) -> base_link=("
                f"{base[0]:+.3f}, {base[1]:+.3f}, {base[2]:+.3f})")

    def _make_depth_msg(self) -> Image:
        msg = self.bridge.cv2_to_imgmsg(self.scene.depth_image(), encoding="16UC1")
        msg.header.frame_id = self.depth_frame
        return msg

    # ------------------------------------------------------------------- publish

    def tick(self):
        # ★★ 한 틱의 모든 메시지가 **같은 stamp**를 씁니다.
        #    실제 계통에서는 이게 원본 이미지의 stamp이고, 태스크②는 그걸 그대로
        #    출력에 실어야 합니다. 여기서 now()를 세 번 부르면 동기화 버그와
        #    stamp 전파 버그가 섞여서 구분이 안 됩니다.
        stamp = self.get_clock().now().to_msg()

        depth_msg = self._depth_msg or self._make_depth_msg()
        depth_msg.header.stamp = stamp      # 내용은 같고 stamp만 갱신
        self.depth_pub.publish(depth_msg)

        self.depth_info_pub.publish(
            self._camera_info(stamp, self.depth_frame, self.scene.k_depth,
                              *self.scene.depth_size))
        self.color_info_pub.publish(
            self._camera_info(stamp, self.color_frame, self.k_enhanced,
                              *self.enh_size))
        self.rgb0_info_pub.publish(
            self._camera_info(stamp, self.color_frame, self.scene.k_color,
                              *self.scene.color_size))

        arr = Detection2DArray()
        det_stamp = stamp
        broken = float(self.get_parameter("break_stamp_sec").value)
        if broken > 0.0:
            total = stamp.sec + stamp.nanosec * 1e-9 + broken
            det_stamp = Time(sec=int(total), nanosec=int((total % 1.0) * 1e9))
        arr.header.stamp = det_stamp
        # 박스는 컬러(=/image_enhanced) 좌표계입니다. frame_id도 컬러 쪽.
        arr.header.frame_id = self.color_frame
        arr.detections.append(self._detection(det_stamp))
        self.det_pub.publish(arr)

    def _detection(self, stamp) -> Detection2D:
        det = Detection2D()
        det.header.stamp = stamp
        det.header.frame_id = self.color_frame

        x1, y1, x2, y2 = self.box_enhanced
        set_bbox_center(det.bbox, (x1 + x2) / 2.0, (y1 + y2) / 2.0)
        det.bbox.size_x = float(x2 - x1)
        det.bbox.size_y = float(y2 - y1)

        result = ObjectHypothesisWithPose()
        set_hypothesis(result, self.class_id, self.score)
        det.results.append(result)
        return det

    def _camera_info(self, stamp, frame_id, k, width, height) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width, info.height = int(width), int(height)
        # 어안이면 backproject 공식이 성립하지 않습니다 (지시서 3-3).
        # 더미는 plumb_bob·무왜곡으로 두고, 실물 값은 로봇 수령 후 확인.
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = list(k)
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [k[0], 0.0, k[2], 0.0,
                  0.0, k[4], k[5], 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info


def main(args=None):
    rclpy.init(args=args)
    node = FakeDetection()
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
