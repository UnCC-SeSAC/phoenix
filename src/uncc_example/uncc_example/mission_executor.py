import functools

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from interfaces.action import SuppressFire
from interfaces.srv import SetString

from .state_manager import StateManager
from .log_utils import make_event_logger

from frontier_exploration_ros2.srv import ControlExploration


class MissionExecutor(Node):

    def __init__(self):
        super().__init__("mission_executor")

        self._event_logger = make_event_logger(self)

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
        # goal 을 보낼 때마다 증가 — 취소된 이전 goal 의 뒤늦은 결과
        # 콜백이 지금 goal 의 상태를 덮어쓰지 못하게 막는 용도.
        self._nav_goal_token = 0

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
            SetString,
            "/state_manager/target_complete",
        )

        # State machine timer
        self.create_timer(
            self.get_parameter("action_check_period").value,
            self.timer_callback,
        )

        # -----------------------------
        # frontier_state_controller.py 에게
        # frontier_explorer START, STOP 신호 보내는 클라이언트
        # -----------------------------
        self.frontier_control_client = self.create_client(
            ControlExploration,
            "/frontier_state_controller/control_exploration",
        )

        # [None, STATE_RUNNING, STATE_IDLE, STATE_STOPPING] 중 하나
        self._frontier_state = None
        self._frontier_request_pending = False

    # =========================================================
    # Subscriptions
    # =========================================================

    def state_callback(self, msg):

        if (
            self.state == StateManager.FIRE_DETECTED
            and msg.data != StateManager.FIRE_DETECTED
        ):
            # FIRE_DETECTED 를 벗어나면 진압이 안 끝났어도 무조건 멈춘다.
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

            # frontier 관련 추가 부분
            if self._frontier_request_pending:
                return

            if self._frontier_state != ControlExploration.Request.STATE_IDLE:
                self._request_frontier_stop()
                return

            self._send_nav_goal(self.current_target)

    # =========================================================
    # EXPLORING
    # =========================================================

    def process_exploring(self):
        self.get_logger().info(
            "[EXPLORING]",
            throttle_duration_sec=1.0,
        )

        # MissionExecutor가 이전에 보낸 fire/person/base goal이 남아 있으면 취소
        self._cancel_nav_goal()

        if self._frontier_request_pending:
            return

        if self._frontier_state == ControlExploration.Request.STATE_RUNNING:
            return

        self._request_frontier_start()

        # if self.target_type == 'frontier':
        #     self._send_nav_goal(self.current_target)
        # else:
        #     # target_type == 'idle' — 아직 갈 곳이 없다.
        #     # self.current_target 에는 예전 frontier 좌표가 그대로
        #     # 남아있을 수 있어 그걸 goal 로 쓰면 안 된다. 진행 중이던
        #     # goal(예: 복귀하다 배터리가 회복된 경우)이 있으면 취소만
        #     # 하고 다음 frontier 를 기다린다.
        #     self._cancel_nav_goal()

    # =========================================================
    # Frontier Controller 관련 함수들
    # =========================================================
    def _request_frontier_control(self, action):
        if self._frontier_request_pending:
            return

        if not self.frontier_control_client.service_is_ready():
            self.get_logger().warn(
                "frontier_state_controller 서비스가 준비되지 않음",
                throttle_duration_sec=2.0,
            )
            return

        request = ControlExploration.Request()
        request.action = action
        request.delay_seconds = 0.0
        request.quit_after_stop = False

        self._frontier_request_pending = True

        future = self.frontier_control_client.call_async(request)
        future.add_done_callback(self._frontier_control_done)

    def _request_frontier_start(self):
        self._request_frontier_control(ControlExploration.Request.ACTION_START)

    def _request_frontier_stop(self):
        self._request_frontier_control(ControlExploration.Request.ACTION_STOP)

    def _frontier_control_done(self, future):
        self._frontier_request_pending = False

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Frontier control failed: {exc}")
            return

        self._frontier_state = response.state

        if not response.accepted:
            self.get_logger().warn(f"Frontier control rejected: {response.message}")
            return

        self.get_logger().info(f"Frontier state={response.state}: {response.message}")

    # =========================================================

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
        self._nav_goal_token += 1
        token = self._nav_goal_token

        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped

        self._event_logger.info(f"Nav2 goal 설정: ({target_xy[0]:.2f}, {target_xy[1]:.2f})")

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(
            functools.partial(self._nav_goal_response, token=token)
        )

    def _cancel_nav_goal(self):

        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None

        self._nav_goal_xy = None
        # 취소만 하고 새 goal 을 안 보내는 경우(EXPLORING 진입)에도, 남아
        # 있던 콜백이 stale 로 인식되도록 토큰을 올려둔다.
        self._nav_goal_token += 1

    def _nav_goal_response(self, future, token):

        if token != self._nav_goal_token:
            # 이미 취소/대체된 goal 의 뒤늦은 응답 — 지금 상태를 건드리지 않는다.
            return

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal 이 거부됨")
            self._nav_goal_xy = None
            return

        self._nav_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            functools.partial(self._nav_goal_result, token=token)
        )

    def _nav_goal_result(self, future, token):

        if token != self._nav_goal_token:
            # 이미 취소/대체된 goal 의 뒤늦은 결과 — unreachable 로 오보하지 않는다.
            return

        self._nav_goal_handle = None

        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._event_logger.info("Nav2 목적지 도착")

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
            self._nav_goal_xy = None
            self.get_logger().warn(
                f"Nav2 goal 이 실패함 — 도달 불가로 보고 다음 목적지로 "
                f"넘어감 (status={status})"
            )
            self.notify_target_complete(status=StateManager.TARGET_STATUS_UNREACHABLE)

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

        self.notify_target_complete(
            status=(
                StateManager.TARGET_STATUS_SUCCESS
                if result.success
                else StateManager.TARGET_STATUS_FAILED
            )
        )

    # =========================================================
    # state_manager 에게 완료 통보
    # =========================================================

    def notify_target_complete(self, status=StateManager.TARGET_STATUS_SUCCESS):

        if not self.target_complete_client.service_is_ready():
            self.get_logger().warn(
                "state_manager/target_complete 서비스가 아직 준비되지 않음"
            )
            return

        future = self.target_complete_client.call_async(SetString.Request(data=status))

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
