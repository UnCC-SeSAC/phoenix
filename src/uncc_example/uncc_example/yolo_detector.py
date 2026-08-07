import json

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloDetector(Node):
    """
    RGB 프레임마다 객체 탐지를 수행해 JSON 으로 publish 한다.

    self.detect() 는 지금은 파이프라인 테스트용 더미 구현이고,
    실제 YOLO 추론 코드(다른 사람이 작성)로 통째로 교체될 예정이다.
    detect() 가 반환하는 dict 형식만 유지되면 나머지(vision_detector,
    state_manager)는 수정할 필요 없다.
    """

    def __init__(self):
        super().__init__('yolo_detector')

        self.declare_parameter('image_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('detections_topic', '/yolo/detections')

        self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_callback,
            1,
        )

        self.detection_pub = self.create_publisher(
            String,
            self.get_parameter('detections_topic').value,
            10,
        )

    def image_callback(self, msg):

        for detection in self.detect(msg):
            out = String()
            out.data = json.dumps(detection)
            self.detection_pub.publish(out)

    def detect(self, image_msg):
        """
        TODO: 실제 YOLO 추론으로 교체 예정.

        지금은 화면 중앙에 fire 하나가 고정으로 잡힌 것처럼 반환해서,
        뒤쪽 vision_detector / state_manager 파이프라인을 실제
        depth 카메라 데이터로 테스트할 수 있게 한다.

        반환 형식: [{class_name, score, box, width, height}, ...]
        box 는 [x_min, y_min, x_max, y_max] 픽셀 좌표.
        """

        width = image_msg.width
        height = image_msg.height

        box_w = width // 8
        box_h = height // 8

        center_x = width // 2
        center_y = height // 2

        return [{
            'class_name': 'fire',
            'score': 1.0,
            'box': [
                center_x - box_w,
                center_y - box_h,
                center_x + box_w,
                center_y + box_h,
            ],
            'width': width,
            'height': height,
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
