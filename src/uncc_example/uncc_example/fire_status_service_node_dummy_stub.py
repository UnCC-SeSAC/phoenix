#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fire_status_service_node 의 통신 테스트용 더미 구현.

실제 fire_status_service_node.py 와 서비스 이름('check_fire_status')·
Request/Response 필드가 동일하므로, fire_suppression_node 입장에서는
실제 노드와 구분되지 않는다. YOLO(/yolo_result_fire) 의존성은 없고,
호출 횟수에 따라 꺼짐/안꺼짐을 흉내만 낸다 — fire_suppression_node
와의 서비스 호출 자체를 검증하기 위한 스텁이다.

fire_suppression_node_dummy_stub.py 의 succeed_on_attempt 패턴과 동일한
방식: succeed_on_call 번째 호출부터 꺼짐으로 응답한다. 카운터는 노드
재시작 전까지 계속 누적된다 (goal 하나짜리 단발 테스트를 기준으로 함).
"""

import rclpy
from rclpy.node import Node

from interfaces.srv import CheckFireStatus


class FireStatusServiceNode(Node):

    def __init__(self):
        super().__init__('fire_status_service_node')

        # 이 번째 호출부터 꺼짐(is_extinguished=True)으로 응답한다.
        # 1이면 매번 성공, 0이면 계속 안꺼짐으로 응답(실패 경로 테스트용).
        self.declare_parameter('succeed_on_call', 2)
        self.declare_parameter('fail_confidence', 0.15)
        self.declare_parameter('success_confidence', 0.92)

        self._call_count = 0

        self.create_service(
            CheckFireStatus, 'check_fire_status', self.on_check_fire_status
        )

        self.get_logger().info(
            'fire_status_service_node (테스트용 더미) 준비 완료 (서비스: check_fire_status)'
        )

    def on_check_fire_status(self, request, response):

        self._call_count += 1

        succeed_on_call = self.get_parameter('succeed_on_call').value
        is_extinguished = (
            succeed_on_call != 0 and self._call_count >= succeed_on_call
        )

        response.is_extinguished = is_extinguished
        response.confidence = self.get_parameter(
            'success_confidence' if is_extinguished else 'fail_confidence'
        ).value

        self.get_logger().info(
            f'{self._call_count}차 판별 요청(더미) -> '
            f'{"꺼짐" if is_extinguished else "안꺼짐"} '
            f'(확신도 {response.confidence:.2f})'
        )

        return response


def main(args=None):

    rclpy.init(args=args)

    node = FireStatusServiceNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
