#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fire_detections_bridge

image_pipeline의 detection_3d_node가 발행하는 /fire/detections(JSON,
class_name/score/x/y/depth — 실제 YOLO26 인식 결과)를
interfaces/msg/ObjectsInfo로 바꿔 /yolo_result_fire에 재발행한다.

fire_status_service_node는 원래 yolov5_ros2의 별도 yolo_detect 인스턴스가
내는 /yolo_result_fire를 구독하도록 짜여 있었다(화재 전용 모델을 따로
띄우는 전제). 지금은 image_pipeline(YOLO26 + 실카메라) 쪽으로 이미 같은
인식을 하고 있으므로, 모델을 두 번 돌리지 않고 같은 감지 결과를
형식만 바꿔 재사용한다. class_name/score는 실제 인식값 그대로 전달한다.

fire_status_service_node는 class_name/score만 보고 box/width/height는
쓰지 않는다 — 여기서는 /fire/detections의 픽셀 중심(x,y)에 고정 크기
박스를 둘러서 채워 넣는다(형식을 맞추기 위한 자리채움일 뿐, 판정에는
영향 없음).
"""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from interfaces.msg import ObjectsInfo, ObjectInfo

INPUT_TOPIC = '/fire/detections'
OUTPUT_TOPIC = '/yolo_result_fire'
BOX_HALF_SIZE = 20  # /fire/detections에는 중심 픽셀만 있어 임의 크기로 감쌈 (판정에 미사용)


class FireDetectionsBridge(Node):
    def __init__(self):
        super().__init__('fire_detections_bridge')

        self.declare_parameter('input_topic', INPUT_TOPIC)
        self.declare_parameter('output_topic', OUTPUT_TOPIC)

        self.sub = self.create_subscription(
            String,
            self.get_parameter('input_topic').value,
            self.on_detections,
            qos_profile_sensor_data,  # detection_3d_node와 동일 QoS (안 맞추면 매칭 실패)
        )
        self.pub = self.create_publisher(
            ObjectsInfo, self.get_parameter('output_topic').value, 10
        )

        self.get_logger().info(
            f'[fire_detections_bridge] {self.get_parameter("input_topic").value} '
            f'-> {self.get_parameter("output_topic").value}'
        )

    def on_detections(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid detection JSON: {e}')
            return

        width, height = payload.get('frame_size', [0, 0])

        objects = []
        for det in payload.get('detections', []):
            obj = ObjectInfo()
            obj.class_name = det.get('class_name', '')
            obj.score = float(det.get('score', 0.0))
            u = int(det.get('x', 0))
            v = int(det.get('y', 0))
            obj.box = [u - BOX_HALF_SIZE, v - BOX_HALF_SIZE,
                       u + BOX_HALF_SIZE, v + BOX_HALF_SIZE]
            obj.width = int(width)
            obj.height = int(height)
            objects.append(obj)

        out = ObjectsInfo()
        out.objects = objects
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FireDetectionsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
