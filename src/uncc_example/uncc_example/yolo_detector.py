import json

import numpy as np

import rclpy

from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloDetector(Node):
    """
    RGB 프레임마다 객체 탐지를 수행하고, depth 이미지에서 해당
    픽셀의 깊이값까지 읽어서, 그 프레임에서 나온 감지 전부를 JSON
    메시지 하나에 리스트로 묶어 publish 한다 (감지별로 따로 안 보냄
    — 뒤쪽에서 순서가 아니라 내용으로 비교/짝짓기할 수 있도록).

    self.detect() 는 지금은 파이프라인 테스트용 더미 구현이고,
    실제 YOLO 추론 코드(다른 사람이 작성)로 통째로 교체될 예정이다.
    detect() 가 반환하는 dict 형식만 유지되면 나머지(vision_detector,
    state_manager)는 수정할 필요 없다.

    depth 읽기(해상도 보정 포함)를 여기서 담당하는 이유는, RGB
    프레임과 depth 프레임을 따로따로 구독해서 나중에(다른 노드에서)
    맞추려고 하면 프레임이 어긋날 수 있기 때문 — 감지 시점에 바로
    같은 프레임 쌍에서 depth 를 읽어 묶어 보낸다. camera_info/TF 를
    이용한 map 좌표 계산 자체는 여전히 vision_detector 가 담당한다.
    """

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter(
            'depth_image_topic', '/depth_cam/depth0/image_raw'
        )
        self.declare_parameter('detections_topic', '/fire/detections')

        # 이 점수 미만인 감지는 아예 publish 하지 않는다
        self.declare_parameter('min_score', 0.5)
        self.min_score = self.get_parameter('min_score').value

        # depth 이미지가 mm(uint16)로 오는 경우의 스케일. float32(m)면 무시
        self.declare_parameter('depth_scale', 0.001)
        self.depth_scale = self.get_parameter('depth_scale').value

        # 픽셀 1개만 읽으면 노이즈(flying pixel)에 그대로 흔들려서
        # 같은 물체가 map 상에서 튀어 두 개로 잡히는 원인이 됨 —
        # (u,v) 주변 이 크기(정사각형, px)의 패치에서 median을 쓴다
        self.declare_parameter('depth_patch_size', 5)
        self.depth_patch_size = self.get_parameter('depth_patch_size').value

        self.bridge = CvBridge()
        self.latest_depth = None

        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            1,
        )

        self.create_subscription(
            Image,
            self.get_parameter('depth_image_topic').value,
            self.depth_callback,
            1,
        )

        self.detection_pub = self.create_publisher(
            String,
            self.get_parameter('detections_topic').value,
            10,
        )

    def depth_callback(self, msg):
        self.latest_depth = msg

    def image_callback(self, msg):

        # 점수 필터부터 먼저 통과시켜서, 살아남는 감지가 하나도
        # 없으면 depth 이미지 디코딩(무거운 작업) 자체를 건너뛴다.
        detections = [
            detection for detection in self.detect(msg)
            if detection['score'] >= self.min_score
        ]

        if not detections or self.latest_depth is None:
            # depth 프레임을 아직 못 받았어도 마찬가지로 건너뛴다
            # (depth 카메라가 잠깐 끊겨도 다음 프레임에서 재시도됨).
            return

        depth_image = self.bridge.imgmsg_to_cv2(
            self.latest_depth,
            desired_encoding='passthrough',
        )
        depth_h, depth_w = depth_image.shape[:2]

        # RGB 해상도는 프레임 전체에서 한 번만 정해지므로, 보정
        # 비율도 감지별로 반복 계산하지 않고 한 번만 구한다.
        scale_x = depth_w / msg.width
        scale_y = depth_h / msg.height
        depth_header = self.latest_depth.header

        # 한 프레임의 감지 결과를 전부 모아서 메시지 하나로 묶어
        # 보낸다 — 감지끼리 순서(도착 순)가 아니라 실제 내용(거리
        # 등)으로 비교/짝짓기할 수 있도록, 프레임 단위로 한꺼번에
        # 전달하기 위함.
        results = []

        for detection in detections:

            result = self._read_depth(
                detection, depth_image, depth_w, depth_h, scale_x, scale_y
            )

            if result is None:
                continue

            u, v, depth_m = result

            results.append({
                'class_name': detection['class_name'],
                # x, y 는 depth 이미지 해상도 기준 픽셀 좌표로
                # 통일해서 보낸다 — vision_detector 는 더 이상
                # RGB/depth 해상도 비율을 몰라도 된다.
                'x': u,
                'y': v,
                'depth': depth_m,
            })

        if not results:
            return

        out = String()
        out.data = json.dumps({
            # 이 프레임의 depth 를 읽은 시각 — vision_detector 가
            # TF 변환할 때 그대로 사용한다. frame_id 는 카메라가
            # 고정 장착이라 vision_detector 쪽 파라미터로 고정.
            'stamp_sec': depth_header.stamp.sec,
            'stamp_nanosec': depth_header.stamp.nanosec,
            'detections': results,
        })
        self.detection_pub.publish(out)

    def _read_depth(
        self, detection, depth_image, depth_w, depth_h, scale_x, scale_y
    ):

        u = int(detection['x'] * scale_x)
        v = int(detection['y'] * scale_y)

        if not (0 <= v < depth_h and 0 <= u < depth_w):
            return None

        half = self.depth_patch_size // 2
        patch = depth_image[
            max(0, v - half):min(depth_h, v + half + 1),
            max(0, u - half):min(depth_w, u + half + 1),
        ].astype(np.float32)

        valid = patch[(patch != 0) & ~np.isnan(patch)]

        if valid.size == 0:
            # 패치 전체가 측정 실패 픽셀
            return None

        # 단일 픽셀 대신 median 사용 — 경계 등에서 튀는 픽셀
        # 하나 때문에 depth 가 흔들려 같은 물체가 map 상에서
        # 두 개로 갈라져 잡히는 걸 막는다
        raw_depth = np.median(valid)

        if depth_image.dtype == np.uint16:
            depth_m = float(raw_depth) * self.depth_scale
        else:
            depth_m = float(raw_depth)

        return u, v, depth_m

    def detect(self, image_msg):
        """
        TODO: 실제 YOLO 추론으로 교체 예정.

        지금은 화면 중앙에 fire 하나가 고정으로 잡힌 것처럼 반환해서,
        뒤쪽 vision_detector / state_manager 파이프라인을 실제
        depth 카메라 데이터로 테스트할 수 있게 한다.

        반환 형식: [{class_name, score, x, y}, ...]
        x, y 는 RGB 이미지 기준 픽셀 좌표 점 하나(바운딩 박스
        아님). RGB 해상도는 image_msg 에서 바로 얻을 수 있으니
        따로 실을 필요 없다.
        """

        return [{
            'class_name': 'fire',
            'score': 1.0,
            'x': image_msg.width // 2,
            'y': image_msg.height // 2,
        }]


def main(args=None):

    rclpy.init(args=args)

    node = YoloDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
