import rclpy

from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.task import Future

from interfaces.action import SuppressFire


DEFAULT_MAX_ATTEMPTS = 3


class FireSuppressionNode(Node):
    """
    fire_suppression_node 의 통신 테스트용 더미 구현.

    팀원 브랜치의 실제 fire_suppression_node.py 와 액션 이름
    ('suppress_fire')·Goal/Feedback/Result 필드가 동일하므로,
    mission_executor 입장에서는 실제 노드와 구분되지 않는다.
    GPIO/YOLO(check_fire_status)/frontier_exploration 같은 하드웨어·
    외부 서비스 의존성은 없고, 시도 횟수에 따라 성공/실패를 흉내만
    낸다 — mission_executor 와의 goal/feedback/result/cancel 통신
    자체를 검증하기 위한 스텁이다.
    """

    def __init__(self):
        super().__init__('fire_suppression_node')

        self.declare_parameter('spray_seconds', 1.0)
        self.declare_parameter('retry_wait_seconds', 0.5)
        # 이 시도째에 성공한 걸로 흉내낸다. 0 이면 계속 실패시켜서
        # max_attempts 소진 경로를 테스트할 수 있다.
        self.declare_parameter('succeed_on_attempt', 2)

        self._action_server = ActionServer(
            self,
            SuppressFire,
            'suppress_fire',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('fire_suppression_node (테스트용 더미) 준비 완료')

    def cancel_callback(self, goal_handle):
        self.get_logger().info('취소 요청 수신')
        return CancelResponse.ACCEPT

    async def _rclpy_sleep(self, seconds):

        future = Future()

        def _on_timer():
            if not future.done():
                future.set_result(None)

        timer = self.create_timer(seconds, _on_timer)
        try:
            await future
        finally:
            timer.cancel()
            self.destroy_timer(timer)

    async def interruptible_sleep(self, goal_handle, seconds, poll_interval=0.1):

        elapsed = 0.0
        while elapsed < seconds:
            if goal_handle.is_cancel_requested:
                return True
            step = min(poll_interval, seconds - elapsed)
            await self._rclpy_sleep(step)
            elapsed += step
        return goal_handle.is_cancel_requested

    async def execute_callback(self, goal_handle):

        max_attempts = goal_handle.request.max_attempts or DEFAULT_MAX_ATTEMPTS
        succeed_on_attempt = self.get_parameter('succeed_on_attempt').value
        spray_seconds = self.get_parameter('spray_seconds').value
        retry_wait_seconds = self.get_parameter('retry_wait_seconds').value

        feedback_msg = SuppressFire.Feedback()
        result = SuppressFire.Result()

        self.get_logger().info(f'진압 시작(더미). 최대 {max_attempts}회 시도.')

        for attempt in range(1, max_attempts + 1):

            if goal_handle.is_cancel_requested:
                return self._handle_cancel(goal_handle, attempt - 1)

            feedback_msg.current_attempt = attempt
            feedback_msg.status = f'{attempt}차 분사 중(더미)'
            goal_handle.publish_feedback(feedback_msg)

            cancelled = await self.interruptible_sleep(goal_handle, spray_seconds)
            if cancelled:
                return self._handle_cancel(goal_handle, attempt)

            feedback_msg.status = f'{attempt}차 상태 확인 중(더미)'
            goal_handle.publish_feedback(feedback_msg)

            if attempt == succeed_on_attempt:
                goal_handle.succeed()
                result.success = True
                result.attempts = attempt
                result.message = '진압 성공(더미)'
                return result

            if attempt < max_attempts:
                cancelled = await self.interruptible_sleep(goal_handle, retry_wait_seconds)
                if cancelled:
                    return self._handle_cancel(goal_handle, attempt)

        goal_handle.succeed()
        result.success = False
        result.attempts = max_attempts
        result.message = '진압 실패(더미, 최대 시도 횟수 도달)'
        return result

    def _handle_cancel(self, goal_handle, attempts_done):

        goal_handle.canceled()

        result = SuppressFire.Result()
        result.success = False
        result.attempts = attempts_done
        result.message = '진압 취소됨(더미)'

        self.get_logger().info(f'취소됨 ({attempts_done}회 시도 후)')

        return result


def main(args=None):

    rclpy.init(args=args)

    node = FireSuppressionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
