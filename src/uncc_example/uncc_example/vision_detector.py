import json
import time

import rclpy

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401  PointStamped 변환 등록용
from tf2_geometry_msgs import do_transform_point

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

from .log_utils import make_event_logger


class VisionDetector(Node):
    """
    yolo_detector 가 프레임 단위로 묶어 publish 하는 JSON(감지
    리스트: class_name + depth 이미지 기준 픽셀 좌표 + depth 값)을
    camera_info/TF 로 map 좌표계 3D 위치로 변환하고, 마찬가지로
    프레임 단위로 묶어서 JSON 으로 다시 publish 한다 — state_manager
    가 감지 도착 순서가 아니라 한 프레임 안의 내용으로 짝짓기할 수
    있도록 순서를 안 흐트러뜨린다.

    depth 이미지를 직접 읽는 건 yolo_detector 가 담당한다(감지
    시점에 바로 같은 프레임에서 depth 를 읽어야 프레임이 안 어긋남).
    여기서는 camera_info(내부 파라미터)+TF(카메라->map) 좌표 변환만
    담당한다.

    동일 객체 판단(dedup/merge)은 여기서도 하지 않는다 —
    state_manager 가 target_merge_radius 기준으로 처리한다.
    """

    def __init__(self):
        super().__init__("vision_detector")

        self._event_logger = make_event_logger(self)

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("map_frame", "map")

        # image_pipeline(fire_suppression 팀 vision 파이프라인)이
        # publish 하는 2D 감지 결과 (JSON, class_name/x/y/depth)
        self.declare_parameter("detections_topic", "/fire/detections")
        # full_chain_dummy_test.launch.py 와 동일한 값 — 카메라 관련
        # 설정은 그 launch 파일 기준을 따른다.
        self.declare_parameter("camera_info_topic", "/image_enhanced/camera_info")

        # 카메라가 로봇에 고정 장착이라 프레임 이름이 항상 같음 —
        # yolo_detector 가 매 메시지마다 안 보내고 여기서 고정값으로 둔다
        self.declare_parameter("depth_frame_id", "ascamera_color_0")

        # 이 클래스들만 fire/person 감지로 취급
        self.declare_parameter("target_classes", ["fire", "person"])

        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("tf_fallback_max_age_sec", 0.5)

        self.map_frame = self.get_parameter("map_frame").value
        self.depth_frame_id = self.get_parameter("depth_frame_id").value
        self.tf_timeout_sec = self.get_parameter("tf_timeout_sec").value
        self.target_classes = set(self.get_parameter("target_classes").value)
        self.tf_fallback_max_age_sec = self.get_parameter("tf_fallback_max_age_sec").value

        self.latest_camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # -----------------------------
        # TF (카메라 좌표 -> map 좌표 변환용)
        # -----------------------------
        self.tf_buffer = Buffer()
        # TF listener uses its own reentrant callback group. The main executor's
        # second worker can receive TF while the detection callback waits for it.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self._tf_counts = dict(exact=0, fallback=0, dropped=0, unavailable=0)
        self._tf_wait_ms = 0.0
        self.create_timer(5.0, self._report_tf_health)

        # -----------------------------
        # Subscriptions
        # -----------------------------
        # image_pipeline 쪽 publisher(camera_info/detections 둘 다)가
        # qos_profile_sensor_data(BEST_EFFORT)로 발행하므로, 여기서
        # 기본(RELIABLE) QoS로 구독하면 QoS 불일치로 아예 매칭이 안 된다.
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            String,
            self.get_parameter("detections_topic").value,
            self.detections_callback,
            qos_profile_sensor_data,
        )

        # -----------------------------
        # Publisher (state_manager 가 구독)
        # -----------------------------
        self.detection_pub = self.create_publisher(
            String,
            "/vision/detections",
            10,
        )

    # =========================================================
    # Camera inputs
    # =========================================================

    def camera_info_callback(self, msg):
        # 카메라 내부 파라미터는 캘리브레이션 후 고정값이라, 감지마다
        # 다시 꺼내 쓰지 않고 CameraInfo 가 갱신될 때만 뽑아둔다.
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.latest_camera_info = msg

    # =========================================================
    # YOLO 2D detections (JSON, depth 포함) -> map 좌표 3D 위치
    # -> JSON publish
    # =========================================================

    def detections_callback(self, msg):

        if self.latest_camera_info is None:
            return

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._event_logger.warn(f"Invalid detection JSON: {e}")
            return

        # 프레임 전체가 같은 시각을 공유하므로 한 번만 변환해둔다.
        stamp = Time(
            seconds=payload["stamp_sec"],
            nanoseconds=payload["stamp_nanosec"],
        ).to_msg()

        results = []
        detections = [d for d in payload.get("detections", [])
                      if d.get("class_name") in self.target_classes
                      and d.get("depth") is not None]
        if not detections:
            return
        transform = self._lookup_frame_transform(stamp)
        if transform is None:
            return

        for detection in detections:

            class_name = detection.get("class_name")

            map_point = self._compute_map_position(detection, stamp, transform)

            if map_point is None:
                continue

            results.append(
                {
                    "class": class_name,
                    "x": map_point.point.x,
                    "y": map_point.point.y,
                }
            )

        if results:
            self._publish_detections(results)

    def _compute_map_position(self, detection, stamp, transform):

        u = detection["x"]
        v = detection["y"]
        depth_m = detection["depth"]

        if depth_m is None:
            return None

        point = PointStamped()
        point.header.frame_id = self.depth_frame_id
        point.header.stamp = stamp
        point.point.x = (u - self.cx) * depth_m / self.fx
        point.point.y = (v - self.cy) * depth_m / self.fy
        point.point.z = depth_m
        return do_transform_point(point, transform)

    def _lookup_frame_transform(self, stamp):
        point = PointStamped()
        point.header.frame_id = self.depth_frame_id
        point.header.stamp = stamp
        started = time.monotonic()
        try:
            result = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.depth_frame_id,
                Time.from_msg(stamp),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
            self._tf_counts['exact'] += 1
            return result

        except TransformException as e:
            return self._transform_with_latest_tf(point, e)
        finally:
            self._tf_wait_ms = (time.monotonic() - started) * 1000.0

    def _transform_with_latest_tf(self, point, first_error):
        """detection 시각의 TF가 없을 때 최신 TransformStamped를 반환한다.
        TF가 밀린 정도가
        tf_fallback_max_age_sec 이내면 최신 TF로 근사한다.
        (로봇이 움직이는 중이므로 오차가 그 시간만큼 쌓인다 —
        너무 밀린 detection 은 위치가 크게 틀리므로 버린다.)"""

        try:
            latest = self.tf_buffer.lookup_transform(
                self.map_frame,
                point.header.frame_id,
                Time(),  # 0 = 가장 최근 TF
            )

        except TransformException as e:
            self._tf_counts['unavailable'] += 1
            self._event_logger.warn(
                f"TF unavailable (detection time: {first_error}, latest: {e})",
                throttle_duration_sec=2.0,
            )
            return None

        # Positive gap means the buffered transform is older than the image.
        gap_sec = (
            Time.from_msg(point.header.stamp).nanoseconds
            - Time.from_msg(latest.header.stamp).nanoseconds
        ) / 1e9

        if abs(gap_sec) > self.tf_fallback_max_age_sec:
            self._tf_counts['dropped'] += 1
            detection_age = (
                self.get_clock().now().nanoseconds
                - Time.from_msg(point.header.stamp).nanoseconds
            ) / 1e9
            self._event_logger.warn(
                f"Detection dropped: gap {gap_sec:+.2f}s "
                f"({'TF stale' if gap_sec > 0 else 'detection stale'}); "
                f"detection_age={detection_age:.3f}s; "
                f"first_error={first_error}",
                throttle_duration_sec=2.0,
            )
            return None

        # 성공 로그도 같은 형식으로
        self._event_logger.warn(
            f"Using latest TF (gap {gap_sec:+.2f}s) for detection",
            throttle_duration_sec=2.0,
        )

        self._tf_counts['fallback'] += 1
        return latest

    def _report_tf_health(self):
        # Nonblocking edge lookups distinguish upstream TF lag from image age.
        now_ns = self.get_clock().now().nanoseconds
        edges = []
        for target, source in ((self.map_frame, 'odom'),
                               ('odom', 'base_footprint'),
                               (self.map_frame, self.depth_frame_id)):
            try:
                tf = self.tf_buffer.lookup_transform(target, source, Time())
                age = (now_ns - Time.from_msg(tf.header.stamp).nanoseconds) / 1e9
                edges.append(f'{target}<-{source}:age={age:.3f}s')
            except TransformException as exc:
                edges.append(f'{target}<-{source}:unavailable={exc}')
        self._event_logger.info(
            f'TF health cumulative={self._tf_counts} last_lookup_ms={self._tf_wait_ms:.1f} '
            f'executor={type(self.executor).__name__}; ' + '; '.join(edges))

    def _publish_detections(self, results):

        payload = {
            "frame_id": self.map_frame,
            "detections": results,
        }

        msg = String()
        msg.data = json.dumps(payload)

        self.detection_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = VisionDetector()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
