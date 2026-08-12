#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
가짜 YOLO 화재 탐지 퍼블리셔 - 테스트 전용.

실제 카메라/모델 없이 /yolo_result_fire 토픽에
interfaces/msg/ObjectsInfo 메시지를 발행해서 fire_status_service_node
(그리고 그 뒤의 fire_suppression_node 재시도 루프)를 테스트한다.

사용:
    source ~/ros2_ws/install/setup.bash
    python3 fake_yolo_publisher.py

    Enter 키: 불꽃 감지 on/off 토글
    Ctrl+C  : 종료
"""

import threading

import rclpy
from rclpy.node import Node

from interfaces.msg import ObjectsInfo, ObjectInfo

FIRE_DETECTION_TOPIC = 'yolo_result_fire'
PUBLISH_HZ = 5.0  # 실제 추론 주기를 흉내


class FakeYoloPublisher(Node):
    def __init__(self):
        super().__init__('fake_yolo_publisher')
        self.pub = self.create_publisher(ObjectsInfo, FIRE_DETECTION_TOPIC, 10)
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.publish_tick)
        self.fire_on = True
        self.get_logger().info(
            f"가짜 YOLO 시작. 초기 상태: 불꽃 {'감지' if self.fire_on else '없음'} "
            f"(토픽: {FIRE_DETECTION_TOPIC})"
        )

    def publish_tick(self):
        msg = ObjectsInfo()
        if self.fire_on:
            obj = ObjectInfo()
            obj.class_name = 'fire'
            obj.score = 0.9
            obj.box = [100, 100, 200, 200]
            obj.width = 640
            obj.height = 480
            msg.objects = [obj]
        else:
            msg.objects = []
        self.pub.publish(msg)

    def toggle(self):
        self.fire_on = not self.fire_on
        self.get_logger().info(f"상태 전환 -> 불꽃 {'감지' if self.fire_on else '없음'}")


def main():
    rclpy.init()
    node = FakeYoloPublisher()

    def input_loop():
        while rclpy.ok():
            try:
                input()
            except EOFError:
                break
            node.toggle()

    threading.Thread(target=input_loop, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
