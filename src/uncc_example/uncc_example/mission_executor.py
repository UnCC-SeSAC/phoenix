import rclpy

from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

from .state_manager import StateManager


class MissionExecutor(Node):

    def __init__(self):
        super().__init__('mission_executor')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('action_check_period', 0.2)

        # -----------------------------
        # State (state_manager 로부터 받은 값)
        # -----------------------------
        self.state = None
        self.target_type = None
        self.current_target = None

        # -----------------------------
        # Subscriptions
        # -----------------------------
        self.create_subscription(
            String,
            '/mission/state',
            self.state_callback,
            10,
        )

        self.create_subscription(
            String,
            '/mission/target_type',
            self.target_type_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            '/mission/current_target',
            self.target_callback,
            10,
        )

        # -----------------------------
        # state_manager 에게 현재 목적지 처리가 끝났음을 알리는 클라이언트
        # -----------------------------
        self.target_complete_client = self.create_client(
            Trigger,
            '/state_manager/target_complete',
        )

        # State machine timer
        self.create_timer(
            self.get_parameter('action_check_period').value,
            self.timer_callback,
        )

    # =========================================================
    # Subscriptions
    # =========================================================

    def state_callback(self, msg):
        self.state = msg.data

    def target_type_callback(self, msg):
        self.target_type = msg.data

    def target_callback(self, msg):
        self.current_target = msg

    # =========================================================
    # Timer / State machine
    # =========================================================

    def timer_callback(self):

        if self.state == StateManager.EXPLORING:
            self.process_exploring()

        elif self.state == StateManager.PERSON_DETECTED:
            self.process_person_detected()

        elif self.state == StateManager.FIRE_DETECTED:
            self.process_fire_detected()

        elif self.state == StateManager.RETURNING_TO_BASE:
            self.process_returning_to_base()

    # =========================================================
    # EXPLORING
    # =========================================================

    def process_exploring(self):
        # TODO: self.current_target 을 Nav2 목적지로 보내 미탐사 지역 탐사
        pass

    # =========================================================
    # PERSON_DETECTED
    # =========================================================

    def process_person_detected(self):
        # TODO: self.current_target 으로 접근 후 구조 동작 수행
        # 완료되면 self.notify_target_complete() 호출
        pass

    # =========================================================
    # FIRE_DETECTED
    # =========================================================

    def process_fire_detected(self):
        # TODO: self.current_target 으로 접근 후 진화 동작 수행
        # 완료되면 self.notify_target_complete() 호출
        pass

    # =========================================================
    # RETURNING_TO_BASE
    # =========================================================

    def process_returning_to_base(self):
        # TODO: self.current_target(충전 도크)으로 복귀 후 도킹 동작 수행
        pass

    # =========================================================
    # state_manager 에게 완료 통보
    # =========================================================

    def notify_target_complete(self):

        if not self.target_complete_client.service_is_ready():
            return

        future = self.target_complete_client.call_async(
            Trigger.Request()
        )

        future.add_done_callback(
            self.target_complete_done
        )

    def target_complete_done(self, future):

        try:
            response = future.result()

        except Exception as e:
            self.get_logger().error(
                f'target_complete failed: {e}'
            )
            return

        if not response.success:
            self.get_logger().error(
                'state_manager rejected target_complete'
            )


def main(args=None):

    rclpy.init(args=args)

    node = MissionExecutor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
