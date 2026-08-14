import json
import math

import rclpy

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401  PointStamped 변환 등록용

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped


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
        super().__init__('vision_detector')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('map_frame', 'map')

        # image_pipeline(fire_suppression 팀 vision 파이프라인)이
        # publish 하는 2D 감지 결과 (JSON, class_name/x/y/depth)
        self.declare_parameter(
            'detections_topic', '/fire/detections'
        )
        # full_chain_dummy_test.launch.py 와 동일한 값 — 카메라 관련
        # 설정은 그 launch 파일 기준을 따른다.
        self.declare_parameter(
            'camera_info_topic', '/ascamera/camera_publisher/rgb0/camera_info'
        )

        # 카메라가 로봇에 고정 장착이라 프레임 이름이 항상 같음 —
        # yolo_detector 가 매 메시지마다 안 보내고 여기서 고정값으로 둔다
        self.declare_parameter(
            'depth_frame_id', 'camera_depth_optical_frame'
        )

        # 이 클래스들만 fire/person 감지로 취급
        self.declare_parameter('target_classes', ['fire', 'person'])

        self.map_frame = self.get_parameter('map_frame').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value
        self.target_classes = set(
            self.get_parameter('target_classes').value
        )

        self.latest_camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # -----------------------------
        # TF (카메라 좌표 -> map 좌표 변환용)
        # -----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------
        # Subscriptions
        # -----------------------------
        # image_pipeline 쪽 publisher(camera_info/detections 둘 다)가
        # qos_profile_sensor_data(BEST_EFFORT)로 발행하므로, 여기서
        # 기본(RELIABLE) QoS로 구독하면 QoS 불일치로 아예 매칭이 안 된다.
        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            String,
            self.get_parameter('detections_topic').value,
            self.detections_callback,
            qos_profile_sensor_data,
        )

        # -----------------------------
        # Publisher (state_manager 가 구독)
        # -----------------------------
        self.detection_pub = self.create_publisher(
            String,
            '/vision/detections',
            10,
        )

    # =========================================================
    # Camera inputs
    # =========================================================

    def camera_info_callback(self, msg):
        try:
            fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
        except (AttributeError, IndexError, TypeError):
            self.latest_camera_info = None
            return
        if (
            not all(math.isfinite(value) for value in (fx, fy, cx, cy))
            or fx <= 0.0
            or fy <= 0.0
        ):
            self.latest_camera_info = None
            return
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.latest_camera_info = msg

    def detections_callback(self, msg):
        if self.latest_camera_info is None:
            return

        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as exc:
            self.get_logger().warn(f'Invalid detection JSON: {exc}')
            return
        if not isinstance(payload, dict):
            return

        stamp_sec = payload.get('stamp_sec')
        stamp_nanosec = payload.get('stamp_nanosec')
        if (
            isinstance(stamp_sec, bool)
            or not isinstance(stamp_sec, int)
            or isinstance(stamp_nanosec, bool)
            or not isinstance(stamp_nanosec, int)
            or stamp_sec < 0
            or not 0 <= stamp_nanosec < 1_000_000_000
        ):
            return

        detections = payload.get('detections')
        if not isinstance(detections, list):
            return

        stamp = Time(
            seconds=stamp_sec,
            nanoseconds=stamp_nanosec,
        ).to_msg()
        results = []
        for detection in detections:
            converted = self._convert_detection(detection, stamp)
            if converted is not None:
                results.append(converted)

        if results:
            self._publish_detections(results, stamp_sec, stamp_nanosec)

    def _convert_detection(self, detection, stamp):
        if not isinstance(detection, dict):
            return None

        class_name = detection.get('class_name')
        if class_name not in self.target_classes:
            return None

        try:
            confidence = detection['score']
            u = detection['x']
            v = detection['y']
            depth_m = detection['depth']
        except (KeyError, TypeError):
            return None

        numeric_values = (confidence, u, v, depth_m)
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in numeric_values
        ):
            return None
        if not 0.0 <= confidence <= 1.0 or depth_m <= 0.0:
            return None
        if detection.get('depth_status') not in {
            'ok',
            'fallback_bottom',
            'fallback_below',
            'fallback_ring',
        }:
            return None

        map_point = self._compute_map_position(detection, stamp)
        if map_point is None:
            return None

        return {
            'class': class_name,
            'confidence': confidence,
            'x': map_point.point.x,
            'y': map_point.point.y,
        }

    def _compute_map_position(self, detection, stamp):
        u = detection['x']
        v = detection['y']
        depth_m = detection['depth']

        point = PointStamped()
        point.header.frame_id = self.depth_frame_id
        point.header.stamp = stamp
        point.point.x = (u - self.cx) * depth_m / self.fx
        point.point.y = (v - self.cy) * depth_m / self.fy
        point.point.z = depth_m

        try:
            return self.tf_buffer.transform(
                point,
                self.map_frame,
                timeout=Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(f'TF transform failed: {exc}')
            return None

    def _publish_detections(self, results, stamp_sec, stamp_nanosec):
        payload = {
            'frame_id': self.map_frame,
            'stamp_sec': stamp_sec,
            'stamp_nanosec': stamp_nanosec,
            'detections': results,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.detection_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = VisionDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
