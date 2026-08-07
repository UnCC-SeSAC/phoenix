import json
import math

import numpy as np

import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401  PointStamped 변환 등록용

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped


class VisionDetector(Node):
    """
    yolo_detector 가 publish 하는 JSON 2D 감지(class_name + pixel 좌표점)를
    depth 이미지와 결합해 map 좌표계 3D 위치를 계산하고 JSON 으로
    다시 publish 한다.

    좌표 변환(depth -> map)은 의도적으로 여기서 담당한다 — YOLO 쪽은
    나중에 다른 사람 코드로 교체될 예정이라 TF/카메라 캘리브레이션 같은
    시스템 통합 로직을 그쪽에 두지 않는다.

    동일 객체 판단(dedup/merge)은 여기서도 하지 않는다 —
    state_manager 가 target_merge_radius 기준으로 처리한다.
    """

    def __init__(self):
        super().__init__('vision_detector')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('map_frame', 'map')

        # yolo_detector 가 publish 하는 2D 감지 결과 (JSON)
        self.declare_parameter(
            'detections_topic', '/yolo/detections'
        )
        self.declare_parameter(
            'depth_image_topic', '/depth_cam/depth0/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic', '/depth_cam/rgb/camera_info'
        )

        # 이 클래스들만 fire/person 감지로 취급
        self.declare_parameter('target_classes', ['fire', 'person'])
        self.declare_parameter('min_score', 0.5)

        # depth 이미지가 mm(uint16)로 오는 경우의 스케일. float32(m)면 무시
        self.declare_parameter('depth_scale', 0.001)

        self.map_frame = self.get_parameter('map_frame').value
        self.target_classes = set(
            self.get_parameter('target_classes').value
        )
        self.min_score = self.get_parameter('min_score').value
        self.depth_scale = self.get_parameter('depth_scale').value

        self.bridge = CvBridge()

        self.latest_depth = None
        self.latest_camera_info = None

        # -----------------------------
        # TF (카메라 좌표 -> map 좌표 변환용)
        # -----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------
        # Subscriptions
        # -----------------------------
        self.create_subscription(
            Image,
            self.get_parameter('depth_image_topic').value,
            self.depth_callback,
            1,
        )

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            1,
        )

        self.create_subscription(
            String,
            self.get_parameter('detections_topic').value,
            self.detections_callback,
            5,
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

    def depth_callback(self, msg):
        self.latest_depth = msg

    def camera_info_callback(self, msg):
        self.latest_camera_info = msg

    # =========================================================
    # YOLO 2D detections (JSON) -> map 좌표 3D 위치 -> JSON publish
    # =========================================================

    def detections_callback(self, msg):

        if self.latest_depth is None or self.latest_camera_info is None:
            return

        try:
            detection = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(
                f'Invalid detection JSON: {e}'
            )
            return

        class_name = detection.get('class_name')

        if class_name not in self.target_classes:
            return

        if detection.get('score', 0.0) < self.min_score:
            return

        map_point = self._compute_map_position(detection)

        if map_point is None:
            return

        self._publish_detection(class_name, map_point)

    def _compute_map_position(self, detection):

        depth_image = self.bridge.imgmsg_to_cv2(
            self.latest_depth,
            desired_encoding='passthrough',
        )

        # x, y 는 RGB 해상도 기준 픽셀 좌표라, depth 해상도가 다르면
        # 비율로 맞춘다
        depth_h, depth_w = depth_image.shape[:2]
        scale_x = depth_w / detection['width']
        scale_y = depth_h / detection['height']

        u = int(detection['x'] * scale_x)
        v = int(detection['y'] * scale_y)

        if not (0 <= v < depth_h and 0 <= u < depth_w):
            return None

        raw_depth = depth_image[v, u]

        if raw_depth == 0 or math.isnan(float(raw_depth)):
            # 측정 실패 픽셀
            return None

        if depth_image.dtype == np.uint16:
            depth_m = float(raw_depth) * self.depth_scale
        else:
            depth_m = float(raw_depth)

        camera_info = self.latest_camera_info
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        point = PointStamped()
        point.header = self.latest_depth.header
        point.point.x = (u - cx) * depth_m / fx
        point.point.y = (v - cy) * depth_m / fy
        point.point.z = depth_m

        try:
            return self.tf_buffer.transform(
                point,
                self.map_frame,
                timeout=Duration(seconds=0.2),
            )

        except TransformException as e:
            self.get_logger().warn(
                f'TF transform failed: {e}'
            )
            return None

    def _publish_detection(self, class_name, map_point):

        payload = {
            'class': class_name,
            'x': map_point.point.x,
            'y': map_point.point.y,
            'z': map_point.point.z,
            'frame_id': self.map_frame,
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
