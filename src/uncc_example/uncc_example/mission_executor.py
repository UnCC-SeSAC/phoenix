import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from interfaces.action import SuppressFire

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

        # -----------------------------
        # 진압 동작 (fire_suppression_node)
        # -----------------------------
        # fire_suppression_node 는 액션 서버를 네임스페이스 없이
        # 'suppress_fire' 로 등록하므로 클라이언트 이름도 똑같이 맞춘다
        # (launch 시 한쪽에만 네임스페이스가 붙으면 서로 못 찾는다).
        self._fire_action_client = ActionClient(
            self,
            SuppressFire,
            "suppress_fire",
        )

        self._fire_goal_handle = None
        # goal 수락 응답 오기 전에 취소 요청이 오면 여기 남겨뒀다가
        # 수락되는 즉시 취소한다 (안 그러면 그 사이 취소 요청이 씹힘).
        self._fire_cancel_pending = False

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
            PoseStamped,
            "/mission/current_target",
            self.target_callback,
            10,
        )

        # -----------------------------
        # state_manager 에게 현재 목적지 처리가 끝났음을 알리는 클라이언트
        # -----------------------------
        self.target_complete_client = self.create_client(
            SetBool,
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

        if msg.data == StateManager.RETURNING_TO_CHARGE:
            # 배터리 부족 등으로 복귀가 최우선이 되면, 진행 중이던
            # 진압 동작은 더 이상 의미가 없으니 취소한다.
            self._cancel_fire_suppression()

        self.state = msg.data

    def target_callback(self, msg):
        self.current_target = msg

    # =========================================================
    # Timer / State machine
    # =========================================================

    def timer_callback(self):

        if self.state == StateManager.EXPLORING:
            self.process_exploring()

        elif self.state in (
            StateManager.PERSON_DETECTED,
            StateManager.FIRE_DETECTED,
            StateManager.RETURNING_TO_CHARGE,
        ):
            # 셋 다 "현재 목적지로 이동"까지는 동일하게 처리한다. 도착 후
            # 동작(구조/진압 호출/복귀 완료)이 갈리는 부분은 _nav_goal_result
            # 에서 state 로 분기한다.
            self.get_logger().info(
                f"[{self.state}] target={self._target_str()}",
                throttle_duration_sec=1.0,
            )
            self._send_nav_goal(self.current_target)

    # =========================================================
    # EXPLORING
    # =========================================================

    def process_exploring(self):
        self.get_logger().info(
            "[EXPLORING]",
            throttle_duration_sec=1.0,
        )

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

            # 도착 시점의 state 로 분기한다 — target 이 진행 중인 동안엔
            # state_manager 가 state 를 안 바꾸므로 이 값을 그대로 믿어도 된다.
            if self.state == StateManager.FIRE_DETECTED:
                # 도착만으론 완료가 아니다 — 진압 노드를 불러서 그
                # 응답이 와야 완료 처리한다.
                self._call_fire_suppression()
            else:
                # RETURNING_TO_CHARGE/EXPLORING 은 active_target 이 없어서
                # 호출해도 무시되고(no-op), person 은 이걸로 완료 처리된다.
                self.notify_target_complete()
        else:
            self.get_logger().warn(f"Nav2 goal 이 완료되지 못함 (status={status})")

    # =========================================================
    # 진압 동작 (fire_suppression_node 호출)
    # =========================================================

    def _call_fire_suppression(self):
        """suppress_fire 액션에 goal 을 보낸다. 완료 처리는 여기서
        기다리지 않고 _suppress_goal_result 에서 한다."""

        if not self._fire_action_client.server_is_ready():
            self.get_logger().warn(
                "fire_suppression 액션 서버가 아직 준비되지 않음",
                throttle_duration_sec=2.0,
            )
            return

        self._fire_cancel_pending = False

        goal_msg = SuppressFire.Goal()
        # max_attempts 를 안 채우면(0) 서버가 자체 DEFAULT_MAX_ATTEMPTS 를 쓴다.

        send_future = self._fire_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._suppress_feedback_callback,
        )
        send_future.add_done_callback(self._suppress_goal_response)

    def _cancel_fire_suppression(self):

        if self._fire_goal_handle is not None:
            self._fire_goal_handle.cancel_goal_async()
            self._fire_goal_handle = None
        else:
            # goal 을 보냈지만 아직 수락 응답이 안 왔을 수도 있다 —
            # _suppress_goal_response 에서 수락되는 즉시 취소하도록 남겨둔다.
            self._fire_cancel_pending = True

    def _suppress_feedback_callback(self, feedback_msg):

        fb = feedback_msg.feedback

        self.get_logger().info(
            f"fire_suppression {fb.current_attempt}차 — {fb.status}",
            throttle_duration_sec=1.0,
        )

    def _suppress_goal_response(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("fire_suppression goal 이 거부됨")
            return

        if self._fire_cancel_pending:
            self._fire_cancel_pending = False
            goal_handle.cancel_goal_async()

        self._fire_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._suppress_goal_result)

    def _suppress_goal_result(self, future):
        """성공 여부와 상관없이 결과가 오면 완료 처리하고 다음 목적지로
        넘어간다(재시도 없음). 실제 성공 여부는 state_manager 에 전달해서
        지도 표시(꺼짐/안꺼짐)에 반영되게 한다."""

        self._fire_goal_handle = None

        result = future.result().result

        if not result.success:
            self.get_logger().warn(
                f"fire_suppression 실패(attempts={result.attempts}): "
                f"{result.message} — 그래도 넘어감"
            )
        else:
            self.get_logger().info(
                f"fire_suppression 성공(attempts={result.attempts}): "
                f"{result.message}"
            )

        self.notify_target_complete(success=result.success)

    # =========================================================
    # state_manager 에게 완료 통보
    # =========================================================

    def notify_target_complete(self, success=True):

        if not self.target_complete_client.service_is_ready():
            self.get_logger().warn(
                "state_manager/target_complete 서비스가 아직 준비되지 않음"
            )
            return

        future = self.target_complete_client.call_async(
            SetBool.Request(data=success)
        )

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
