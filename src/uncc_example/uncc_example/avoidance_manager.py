import math
import time

import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus, GoalStatusArray

from std_srvs.srv import Trigger
from interfaces.srv import SetInt64
from frontier_exploration_ros2.srv import ControlExploration


class AvoidanceManager(Node):

    NORMAL = "NORMAL"  # Frontier Exploration 이 정상 동작
    STOPPING_FRONTIER = "STOPPING_FRONTIER"
    STARTING_AVOIDANCE = "STARTING_AVOIDANCE"
    AVOIDING = "AVOIDING"
    STOPPING_AVOIDANCE = "STOPPING_AVOIDANCE"
    STARTING_FRONTIER = "STARTING_FRONTIER"

    def __init__(self):
        super().__init__("avoidance_manager")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter(
            "trigger_distance", 0.3
        )  # 이 거리안에 장애물이 들어오면 회피
        self.declare_parameter(
            "clear_distance", 0.35
        )  # 이 거리 이상 확보되면 장애물이 없어졌다고 판단
        self.declare_parameter("front_angle_deg", 210.0)  # 로봇 전방 몇 도를 검사할지
        self.declare_parameter(
            "clear_hold_sec", 0.60
        )  # 안전거리가 얼마나 지속돼야 회피를 끝낼 것인지
        self.declare_parameter(
            "avoidance_timeout_sec", 5.0
        )  # 회피를 무한히 하지 않도록 최대 시간

        self.trigger_distance = self.get_parameter("trigger_distance").value
        self.clear_distance = self.get_parameter("clear_distance").value
        self.front_angle_deg = self.get_parameter("front_angle_deg").value
        self.clear_hold_sec = self.get_parameter("clear_hold_sec").value
        self.avoidance_timeout_sec = self.get_parameter("avoidance_timeout_sec").value

        # -----------------------------
        # State
        # -----------------------------
        self.state = self.NORMAL

        self.front_distance = math.inf

        self.nav_goal_active = False  # 현재 Nav2 목적지를 향해 이동 중인가
        self.last_nav_status_time = 0.0

        self.clear_start_time = None
        self.avoidance_start_time = None

        self.lidar_entered = False  # /lidar_app/enter 호출 성공 여부
        self.enter_pending = False

        self.frontier_stop_state = None
        self.frontier_stop_response_time = 0.0
        self.stop_request_pending = False

        # -----------------------------
        # /scan_raw
        # -----------------------------
        scan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(
            LaserScan,
            "/scan_raw",
            self.scan_callback,
            scan_qos,
        )

        # -----------------------------
        # Nav2 Action Status
        # -----------------------------
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.nav_status_callback,
            10,
        )

        # -----------------------------
        # Service Clients
        # -----------------------------

        # /control_exploration 토픽? 을 통해 탐색 멈춤 시작을 조절
        self.frontier_client = self.create_client(
            ControlExploration,
            "/control_exploration",
        )

        # 회피 기능 준비
        self.lidar_enter_client = self.create_client(
            Trigger,
            "/lidar_app/enter",
        )

        # 회피 기능 Start/Stop
        self.lidar_running_client = self.create_client(
            SetInt64,
            "/lidar_app/set_running",
        )

        # State machine timer
        self.create_timer(
            0.05,
            self.timer_callback,
        )

    # =========================================================
    # LaserScan
    # =========================================================

    def scan_callback(self, msg):

        distance = self.get_front_min_distance(msg)

        if distance is None:
            return

        self.front_distance = distance

        # 장애물 발견시 바로 회피 시작이 아닌 Frotier, Nav2 를 중단 후 회피시작
        if (
            self.state == self.NORMAL
            and self.lidar_entered
            and distance <= self.trigger_distance
        ):
            self.get_logger().warn(f"Obstacle detected: {distance:.2f} m")

            #
            self.state = self.STOPPING_FRONTIER
            self.stop_request_pending = False
            self.frontier_stop_state = None

    # 전방 -45 ~ 45 도 영역의  최소 거리 계산, 계산된 최소 거리 반환
    def get_front_min_distance(self, msg):

        half_angle = math.radians(self.front_angle_deg) / 2.0

        min_distance = math.inf
        found = False

        for i, distance in enumerate(msg.ranges):

            # 유한한 수 인지
            if not math.isfinite(distance):
                continue

            # lidar 측정 가능한 min 값 보다 작으면 버림
            if distance < msg.range_min:
                continue

            # lidar 측정 가능한 max 값 보다 크면 버림
            if distance > msg.range_max:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            # -pi ~ +pi
            angle = math.atan2(
                math.sin(angle),
                math.cos(angle),
            )

            if abs(angle) <= half_angle:
                min_distance = min(
                    min_distance,
                    distance,
                )
                found = True

        # 유효한 전방 데이터가 없으면 None 반환
        if not found:
            return None

        return min_distance

    # =========================================================
    # Nav2 status
    # =========================================================

    def nav_status_callback(self, msg):

        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }

        self.nav_goal_active = any(
            status.status in active_states for status in msg.status_list
        )

        self.last_nav_status_time = time.monotonic()

    # =========================================================
    # Timer / State machine
    # =========================================================

    def timer_callback(self):

        # lidar_app enter 자동 실행
        self.ensure_lidar_entered()

        if self.state == self.STOPPING_FRONTIER:
            self.process_frontier_stop()

        elif self.state == self.AVOIDING:
            self.process_avoiding()

    # =========================================================
    # lidar_app enter
    # =========================================================

    def ensure_lidar_entered(self):

        if self.lidar_entered:
            return

        if self.enter_pending:
            return

        if not self.lidar_enter_client.service_is_ready():
            return

        self.enter_pending = True

        future = self.lidar_enter_client.call_async(Trigger.Request())

        future.add_done_callback(self.enter_done)

    def enter_done(self, future):

        self.enter_pending = False

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"lidar enter failed: {e}")
            return

        if response.success:
            self.lidar_entered = True
            self.get_logger().info("lidar_app entered")

    # =========================================================
    # Frontier STOP
    # =========================================================

    def process_frontier_stop(self):

        if not self.stop_request_pending:

            if not self.frontier_client.service_is_ready():
                return

            request = ControlExploration.Request()

            request.action = ControlExploration.Request.ACTION_STOP

            request.delay_seconds = 0.0
            request.quit_after_stop = False

            future = self.frontier_client.call_async(request)

            future.add_done_callback(self.frontier_stop_done)

            self.stop_request_pending = True

            return

        if self.frontier_stop_state is None:
            return

        # 이미 완전히 정지
        if self.frontier_stop_state == ControlExploration.Request.STATE_IDLE:
            self.start_avoidance()
            return

        # Nav2 goal cancel 완료 확인
        if self.frontier_stop_state == ControlExploration.Request.STATE_STOPPING:
            if (
                self.last_nav_status_time > self.frontier_stop_response_time
                and not self.nav_goal_active
            ):
                self.start_avoidance()

    def frontier_stop_done(self, future):

        try:
            response = future.result()

        except Exception as e:
            self.get_logger().error(f"Frontier stop failed: {e}")
            return

        if not response.accepted:
            self.get_logger().error(response.message)
            return

        self.frontier_stop_state = response.state

        self.frontier_stop_response_time = time.monotonic()

    # =========================================================
    # Hiwonder Avoidance START
    # =========================================================

    def start_avoidance(self):

        if self.state != self.STOPPING_FRONTIER:
            return

        if not self.lidar_running_client.service_is_ready():
            return

        self.state = self.STARTING_AVOIDANCE

        request = SetInt64.Request()
        request.data = 1

        future = self.lidar_running_client.call_async(request)

        future.add_done_callback(self.avoidance_start_done)

    def avoidance_start_done(self, future):

        response = future.result()

        if not response.success:
            self.get_logger().error("Failed to start avoidance")
            return

        self.state = self.AVOIDING

        self.avoidance_start_time = time.monotonic()

        self.clear_start_time = None

        self.get_logger().warn("LD19 avoidance owns motion")

    # =========================================================
    # AVOIDING
    # =========================================================

    def process_avoiding(self):

        now = time.monotonic()

        # 무한 회피 방지
        if now - self.avoidance_start_time >= self.avoidance_timeout_sec:
            self.stop_avoidance()
            return

        # hysteresis
        if self.front_distance >= self.clear_distance:

            if self.clear_start_time is None:
                self.clear_start_time = now

            elif now - self.clear_start_time >= self.clear_hold_sec:
                self.stop_avoidance()

        else:
            self.clear_start_time = None

    # =========================================================
    # Hiwonder Avoidance STOP
    # =========================================================

    def stop_avoidance(self):

        if self.state != self.AVOIDING:
            return

        self.state = self.STOPPING_AVOIDANCE

        request = SetInt64.Request()
        request.data = 0

        future = self.lidar_running_client.call_async(request)

        future.add_done_callback(self.avoidance_stop_done)

    def avoidance_stop_done(self, future):

        response = future.result()

        if not response.success:
            self.get_logger().error("Failed to stop avoidance")
            return

        self.start_frontier()

    # =========================================================
    # Frontier START
    # =========================================================

    def start_frontier(self):

        self.state = self.STARTING_FRONTIER

        request = ControlExploration.Request()

        request.action = ControlExploration.Request.ACTION_START

        request.delay_seconds = 0.0
        request.quit_after_stop = False

        future = self.frontier_client.call_async(request)

        future.add_done_callback(self.frontier_start_done)

    def frontier_start_done(self, future):

        response = future.result()

        if not response.accepted:
            self.get_logger().error(response.message)
            return

        self.stop_request_pending = False
        self.frontier_stop_state = None
        self.clear_start_time = None

        self.state = self.NORMAL

        self.get_logger().info("Frontier exploration resumed")


def main(args=None):

    rclpy.init(args=args)

    node = AvoidanceManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
