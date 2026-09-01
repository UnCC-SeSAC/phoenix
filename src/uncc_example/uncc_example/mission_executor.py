import functools
import json
import math

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from tf2_ros import Buffer, TransformListener, TransformException

from builtin_interfaces.msg import Duration as ActionDuration
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose, Spin, DriveOnHeading

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
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")

        # keepout 원 겹침 판정 여유 — footprint 대각선(약 0.195m)만큼
        # 남았을 때 걸치는 걸로 본다 (fire_keepout_node 의
        # registration 여유와 같은 값).
        self.declare_parameter("keepout_escape_margin", 0.20)
        self.declare_parameter("keepout_escape_speed", 0.10)
        self.declare_parameter("keepout_escape_time_allowance", 10.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.keepout_escape_margin = (
            self.get_parameter("keepout_escape_margin").value
        )
        self.keepout_escape_speed = (
            self.get_parameter("keepout_escape_speed").value
        )
        self.keepout_escape_time_allowance = (
            self.get_parameter("keepout_escape_time_allowance").value
        )

        # -----------------------------
        # State (state_manager 로부터 받은 값)
        # -----------------------------
        self.state = None
        self.current_target = None

        # -----------------------------
        # Keepout 이탈 (주행 중 새로 걸친 mask 를 감지해서 빠져나감)
        # -----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._keepout_circles = []  # [(x, y, radius), ...]
        self._escaping = False
        self._escape_distance = 0.0

        self._spin_client = ActionClient(self, Spin, "spin")
        self._drive_client = ActionClient(self, DriveOnHeading, "drive_on_heading")

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

        # fire_keepout_node 와 동일한 QoS(transient_local) 여야 늦게
        # 떠도 마지막 값을 받는다.
        keepout_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            String,
            "/fire_keepout_circles",
            self._keepout_circles_callback,
            keepout_qos,
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

        # halt trace 로그
        if self.state != msg.data:
            self.get_logger().info(f"[MISSION_STATE] {self.state} -> {msg.data}")
        ###

        if (
            self.state == StateManager.FIRE_DETECTED
            and msg.data != StateManager.FIRE_DETECTED
        ):
            # FIRE_DETECTED 를 벗어나면 진압이 안 끝났어도 무조건 멈춘다.
            self._cancel_fire_suppression()

        self.state = msg.data

    def target_callback(self, msg):
        self.current_target = msg

    def _keepout_circles_callback(self, msg):

        try:
            circles = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Invalid fire_keepout_circles JSON: {e}")
            return

        self._keepout_circles = [
            (c["x"], c["y"], c["radius"]) for c in circles
        ]

    # =========================================================
    # Timer / State machine
    # =========================================================

    def timer_callback(self):

        if self._escaping:
            return

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

            # 주행 중 새로 검출된 물체가 경로 위에서 keepout 으로 잡혀
            # 로봇 발판과 겹치는 경우 — nav2 recovery 는 이게 우리가
            # 만든 원이라는 걸 몰라서 방향을 못 잡는다. 우리가 direct
            # 로 방향을 계산해서 빠져나간다.
            if self._nav_goal_handle is not None:
                overlap = self._find_keepout_overlap()

                if overlap is not None:
                    self._start_keepout_escape(overlap)
                    return

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

        # halt trace 로그
        action_name = (
            "START" if action == ControlExploration.Request.ACTION_START else "STOP"
        )

        self.get_logger().info(
            f"[FRONTIER_CONTROL_REQUEST] "
            f"action={action_name}, mission_state={self.state}"
        )
        ###

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

        self._event_logger.info(
            f"Nav2 goal 설정: ({target_xy[0]:.2f}, {target_xy[1]:.2f})"
        )

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
    # Keepout 이탈 — 주행 중 로봇 발판이 keepout 원과 겹치면, 그 원의
    # 중심에서 로봇 쪽으로 향하는 방향으로 회전(Spin) 후 전진
    # (DriveOnHeading) 해서 빠져나간다. 방향을 우리가 직접 계산해서
    # 주는 이유는 nav2 기본 recovery(Spin/BackUp) 는 이게 우리가 만든
    # 원이라는 걸 몰라서 임의 방향으로 시도하기 때문 — 잘못된 방향으로
    # 후퇴하면 안 풀릴 수 있다.
    # =========================================================

    def _robot_pose_yaw(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        return (t.x, t.y, yaw)

    def _find_keepout_overlap(self):

        pose = self._robot_pose_yaw()

        if pose is None:
            return None

        px, py, _ = pose

        for x, y, radius in self._keepout_circles:
            distance = math.hypot(px - x, py - y)

            if distance < radius + self.keepout_escape_margin:
                return (x, y, radius, distance)

        return None

    def _start_keepout_escape(self, overlap):

        x, y, radius, distance = overlap

        pose = self._robot_pose_yaw()

        if pose is None:
            return

        if not self._spin_client.server_is_ready():
            self.get_logger().warn(
                "spin 액션 서버가 아직 준비되지 않음 — keepout 이탈 보류",
                throttle_duration_sec=2.0,
            )
            return

        px, py, yaw = pose

        self._escaping = True
        self._cancel_nav_goal()

        escape_angle = math.atan2(py - y, px - x)
        relative_yaw = math.atan2(
            math.sin(escape_angle - yaw), math.cos(escape_angle - yaw)
        )
        # 여유(margin) + 약간의 버퍼(0.05m)만큼 더 벌어질 때까지 전진.
        self._escape_distance = (
            radius + self.keepout_escape_margin + 0.05 - distance
        )

        self._event_logger.info(
            f"keepout 이탈: ({x:.2f}, {y:.2f}) r={radius:.2f}m 안으로 "
            f"{distance:.2f}m — {math.degrees(relative_yaw):.0f}도 회전 후 "
            f"{self._escape_distance:.2f}m 전진"
        )

        goal = Spin.Goal()
        goal.target_yaw = relative_yaw
        goal.time_allowance = ActionDuration(
            sec=int(self.keepout_escape_time_allowance)
        )

        send_future = self._spin_client.send_goal_async(goal)
        send_future.add_done_callback(self._spin_goal_response)

    def _spin_goal_response(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("keepout 이탈용 spin goal 이 거부됨")
            self._escaping = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._spin_goal_result)

    def _spin_goal_result(self, future):

        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f"keepout 이탈용 spin 실패(status={status}) — 다음 tick 에 재시도"
            )
            self._escaping = False
            return

        if not self._drive_client.server_is_ready():
            self.get_logger().warn("drive_on_heading 액션 서버가 아직 준비되지 않음")
            self._escaping = False
            return

        goal = DriveOnHeading.Goal()
        goal.target = Point(x=self._escape_distance, y=0.0, z=0.0)
        goal.speed = self.keepout_escape_speed
        goal.time_allowance = ActionDuration(
            sec=int(self.keepout_escape_time_allowance)
        )

        send_future = self._drive_client.send_goal_async(goal)
        send_future.add_done_callback(self._drive_goal_response)

    def _drive_goal_response(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("keepout 이탈용 drive_on_heading goal 이 거부됨")
            self._escaping = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._drive_goal_result)

    def _drive_goal_result(self, future):

        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._event_logger.info("keepout 이탈 완료")
        else:
            self.get_logger().warn(
                f"keepout 이탈용 전진 실패(status={status}) — 다음 tick 에 재시도"
            )

        self._escaping = False

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
