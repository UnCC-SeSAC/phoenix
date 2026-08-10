#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fire_extinguisher

mission_executor 팀(mission_executor.py)이 이미 구현해서 호출하고 있는
외부 인터페이스 - Trigger 서비스 '~/extinguish', Bool 토픽 '~/result' -
는 그대로 유지하면서, 내부적으로는 fire_suppression_node가 제공하는
SuppressFire Action을 호출한다.

mission_executor.py 쪽 코드는 이 파일이 어떻게 구현됐든 전혀 몰라도 된다 -
노드 이름이 'fire_extinguisher'로 동일하기만 하면
/fire_extinguisher/extinguish, /fire_extinguisher/result 이름이 그대로
유지된다.

원본 스텁과 달리 추가된 것:
  - _busy 가드: 진압 중 재요청이 오면 즉시 거절
  - 실제 결과는 SuppressFire Action의 result가 온 뒤에야 result_pub으로
    발행됨 (원본처럼 바로 True를 쏘지 않음)

단독 유닛테스트:
    ros2 run uncc_example fire_extinguisher
    ros2 service call /fire_extinguisher/extinguish std_srvs/srv/Trigger "{}"
    ros2 topic echo /fire_extinguisher/result
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from interfaces.action import SuppressFire


class FireExtinguisher(Node):
    def __init__(self):
        super().__init__('fire_extinguisher')

        self._busy = False

        self.result_pub = self.create_publisher(Bool, '~/result', 10)
        self.create_service(Trigger, '~/extinguish', self.extinguish_callback)

        self._action_client = ActionClient(self, SuppressFire, 'suppress_fire')

        self.get_logger().info(
            "fire_extinguisher 준비 완료 "
            "(외부: ~/extinguish, ~/result / 내부: suppress_fire Action)"
        )

    def extinguish_callback(self, request, response):
        if self._busy:
            self.get_logger().warn('이미 진압 작업이 진행 중입니다. 요청을 거절합니다.')
            response.success = False
            response.message = 'already running'
            return response

        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('suppress_fire Action 서버에 연결할 수 없습니다.')
            response.success = False
            response.message = 'suppress_fire action server unavailable'
            return response

        self._busy = True
        self.get_logger().info('불 끄기 동작 시작 (suppress_fire goal 전송)')

        goal_msg = SuppressFire.Goal()
        goal_msg.max_attempts = 0  # 0 -> 서버 기본값(3회) 사용

        send_future = self._action_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

        # Trigger 응답은 "시작 접수됨"이라는 뜻일 뿐, 진화 성공 여부가 아니다.
        # 실제 성공/실패는 나중에 result_pub으로 비동기 통지된다 (기존 설계 유지).
        response.success = True
        response.message = 'suppression started'
        return response

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('suppress_fire goal이 거부되었습니다.')
            self._busy = False
            self.result_pub.publish(Bool(data=False))
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        self.get_logger().info(
            f'진압 결과: success={result.success}, attempts={result.attempts}, '
            f'message={result.message}'
        )
        self.result_pub.publish(Bool(data=result.success))
        self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = FireExtinguisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
