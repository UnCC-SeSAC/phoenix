import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from .state_manager import StateManager


class MissionExecutor(Node):

    def __init__(self):
        super().__init__("mission_executor")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("action_check_period", 0.2)

        # -----------------------------
        # State (state_manager 로부터 받은 값)
        # -----------------------------
        self.state = None
        self.target_type = None
        self.current_target = None

        # -----------------------------
        # Nav2 (fire/person target 로 이동)
        # -----------------------------
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose",
        )

        self._nav_goal_handle = None
        self._nav_goal_xy = None  # 지금 보내둔 goal 좌표 (x, y)
        self._nav_goal_state = None  # 그 goal 을 보낼 때의 self.state

        # -----------------------------
        # 진화 동작 (fire_extinguisher 노드)
        # -----------------------------
        self.fire_extinguisher_client = self.create_client(
            Trigger,
            "/fire_extinguisher/extinguish",
        )

        # ~/extinguish 는 "시작해라" 트리거일 뿐이고, 실제 성공/실패
        # 결과는 이 토픽으로 비동기로 온다. 성공/실패 상관없이
        # 결과가 오면 그냥 완료 처리하고 넘어간다 (재시도 안 함).
        self.create_subscription(
            Bool,
            "/fire_extinguisher/result",
            self.fire_extinguisher_result_callback,
            10,
        )

        # -----------------------------
        # Subscriptions
        # -----------------------------
        self.create_subscription(
            String,
            "/mission/state",
            self.state_callback,
            10,
        )

        self.create_subscription(
            String,
            "/mission/target_type",
            self.target_type_callback,
            10,
        )

        self.create_subscription(
            PoseStamped,
            "/mission/current_target",
            self.target_callback,
            10,
        )

        # -----------------------------
        # state_manager 에게 현재 목적지 처리가 끝났음을 알리는 클라이언트
        # -----------------------------
        self.target_complete_client = self.create_client(
            Trigger,
            "/state_manager/target_complete",
        )

        # State machine timer
        self.create_timer(
            self.get_parameter("action_check_period").value,
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
        self.get_logger().info(
            f"[EXPLORING] target={self._target_str()}",
            throttle_duration_sec=1.0,
        )

        if self.target_type == 'frontier':
            self._send_nav_goal(self.current_target)
        else:
            # target_type == 'idle' — 아직 갈 곳이 없다.
            # self.current_target 에는 예전 frontier 좌표가 그대로
            # 남아있을 수 있어 그걸 goal 로 쓰면 안 된다. 진행 중이던
            # goal(예: 복귀하다 배터리가 회복된 경우)이 있으면 취소만
            # 하고 다음 frontier 를 기다린다.
            self._cancel_nav_goal()

    # =========================================================
    # PERSON_DETECTED
    # =========================================================

    def process_person_detected(self):
        self.get_logger().info(
            f"[PERSON_DETECTED] target={self._target_str()}",
            throttle_duration_sec=1.0,
        )
        self._send_nav_goal(self.current_target)
        # TODO: 도착 후 구조 동작 수행 (지금은 도착만 하면 바로 완료 처리됨)

    # =========================================================
    # FIRE_DETECTED
    # =========================================================

    def process_fire_detected(self):
        self.get_logger().info(
            f"[FIRE_DETECTED] target={self._target_str()}",
            throttle_duration_sec=1.0,
        )
        self._send_nav_goal(self.current_target)
        # 도착하면 _nav_goal_result 가 fire_extinguisher 노드를 불러서
        # 진화 동작을 수행하고, 그 응답이 와야 완료 처리된다.

    # =========================================================
    # RETURNING_TO_BASE
    # =========================================================

    def process_returning_to_base(self):
        self.get_logger().info(
            f"[RETURNING_TO_BASE] target={self._target_str()}",
            throttle_duration_sec=1.0,
        )
        self._send_nav_goal(self.current_target)

    def _target_str(self):

        if self.current_target is None:
            return "None"

        position = self.current_target.pose.position

        return f"({position.x:.2f}, {position.y:.2f})"

    # =========================================================
    # Nav2 목적지 전달 (fire/person 접근)
    # =========================================================

    def _send_nav_goal(self, pose_stamped):

        if pose_stamped is None:
            return

        target_xy = (
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
        )

        if target_xy == self._nav_goal_xy:
            # 같은 목적지로 이미 보내둔 goal 이 진행 중이면 다시 안 보낸다
            return

        if not self._nav_client.server_is_ready():
            self.get_logger().warn(
                "navigate_to_pose 액션 서버가 아직 준비되지 않음",
                throttle_duration_sec=2.0,
            )
            return

        # 목적지가 바뀌었으니 진행 중이던 goal 은 취소하고 새로 보낸다
        self._cancel_nav_goal()

        self._nav_goal_xy = target_xy
        self._nav_goal_state = self.state

        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._nav_goal_response)

    def _cancel_nav_goal(self):

        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

        self._nav_goal_xy = None

    def _nav_goal_response(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal 이 거부됨")
            self._nav_goal_xy = None
            return

        self._nav_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_goal_result)

    def _nav_goal_result(self, future):

        self._nav_goal_handle = None
        self._nav_goal_xy = None

        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Nav2 목적지 도착")

            if self._nav_goal_state == StateManager.FIRE_DETECTED:
                # 도착만으론 완료가 아니다 — 진화 노드를 불러서 그
                # 응답이 와야 완료 처리한다.
                self._call_fire_extinguisher()
            else:
                # RETURNING_TO_BASE/EXPLORING 은 state_manager 쪽에
                # active_target 이 없어서 호출해도 그냥 무시된다
                # (안전한 no-op). person 은 이걸로 완료 처리됨.
                self.notify_target_complete()
        else:
            self.get_logger().warn(f"Nav2 goal 이 완료되지 못함 (status={status})")

    # =========================================================
    # 진화 동작 (fire_extinguisher 노드 호출)
    # =========================================================

    def _call_fire_extinguisher(self):
        """
        ~/extinguish 는 동작을 "시작"만 시키는 트리거다. 완료
        여부는 여기서 기다리지 않고 fire_extinguisher_result_callback
        (토픽 구독)에서 처리한다.
        """

        if not self.fire_extinguisher_client.service_is_ready():
            self.get_logger().warn(
                "fire_extinguisher 서비스가 아직 준비되지 않음"
            )
            return

        self.fire_extinguisher_client.call_async(Trigger.Request())

    def fire_extinguisher_result_callback(self, msg):
        """
        불을 껐든 못 껐든(msg.data 상관없이) 결과가 왔으면 그냥
        완료 처리하고 다음 목적지로 넘어간다 — 재시도하지 않는다.
        """

        if not msg.data:
            self.get_logger().warn(
                "fire_extinguisher 가 실패를 report 함 — 그래도 넘어감"
            )

        self.notify_target_complete()

    # =========================================================
    # state_manager 에게 완료 통보
    # =========================================================

    def notify_target_complete(self):

        if not self.target_complete_client.service_is_ready():
            return

        future = self.target_complete_client.call_async(Trigger.Request())

        future.add_done_callback(self.target_complete_done)

    def target_complete_done(self, future):

        try:
            response = future.result()

        except Exception as e:
            self.get_logger().error(f"target_complete failed: {e}")
            return

        if not response.success:
            self.get_logger().error("state_manager rejected target_complete")


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


if __name__ == "__main__":
    main()
