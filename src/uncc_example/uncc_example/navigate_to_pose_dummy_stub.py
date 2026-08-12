import time

import rclpy

from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose

# mission_executor 의 상태 확인 주기(action_check_period 기본 0.2초)보다
# 짧게 응답하면, goal 완료 -> _nav_goal_xy 초기화 -> 다음 타이머 tick 에서
# 같은 목적지로 goal 재전송이 반복되며 fire_suppression 호출도 여러 번
# 겹쳐서 발생한다. 그 주기보다 확실히 길게 잡아 한 번만 실행되게 한다.
ARRIVAL_DELAY_SECONDS = 0.5


class NavigateToPoseDummyStub(Node):
    """
    navigate_to_pose 통신 테스트용 더미.

    실제 이동은 하지 않고 goal 을 받으면 약간의 지연 후 도착(SUCCEEDED)으로
    처리한다. SLAM/Nav2 스택 없이, mission_executor 가 도착 이후
    로직(fire_suppression 호출 등)까지 제대로 실행하는지 로그로만
    확인하고 싶을 때 쓰는 용도.
    """

    def __init__(self):
        super().__init__('navigate_to_pose_dummy_stub')

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            f'navigate_to_pose_dummy_stub 준비 완료 '
            f'(goal 수신 {ARRIVAL_DELAY_SECONDS}초 후 도착 처리)'
        )

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):

        pose = goal_handle.request.pose.pose.position

        self.get_logger().info(
            f'목적지 수신(더미): ({pose.x:.2f}, {pose.y:.2f}) '
            f'— {ARRIVAL_DELAY_SECONDS}초 후 도착 처리'
        )

        time.sleep(ARRIVAL_DELAY_SECONDS)

        goal_handle.succeed()

        return NavigateToPose.Result()


def main(args=None):

    rclpy.init(args=args)

    node = NavigateToPoseDummyStub()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
