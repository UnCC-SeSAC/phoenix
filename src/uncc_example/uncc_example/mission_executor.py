import functools
import json
import math
from collections import deque

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    qos_profile_sensor_data,
)

from tf2_ros import Buffer, TransformListener, TransformException

from builtin_interfaces.msg import Duration as ActionDuration
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose, Spin, DriveOnHeading

from interfaces.action import SuppressFire
from interfaces.srv import SetString

from .state_manager import StateManager
from .log_utils import make_event_logger
from .mission_navigation_safety import (
    approach_candidates,
    forward_corridor_is_clear,
    front_scan_is_clear,
    occupancy_disk_is_clear,
)

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
        # mask 임시 해제 요청 후 이만큼 안에 확인(circles 갱신)이 안 오면
        # 포기하고 재시도한다 — fire_keepout_node 가 안 떠 있는 경우 대비.
        self.declare_parameter("keepout_suppress_confirm_timeout", 2.0)

        # 사람/불 중심은 keepout lethal cell 이므로 그 바깥의 접근점을
        # Nav2 목표로 사용한다.
        self.declare_parameter("person_approach_distance", 0.50)
        self.declare_parameter("fire_approach_distance", 0.45)
        self.declare_parameter("approach_clearance_radius", 0.20)
        self.declare_parameter("costmap_occupied_threshold", 90)

        # 사람/불 NavigateToPose 전용 회전 진동 recovery. frontier goal의
        # recovery와 같은 판정값을 쓰되 mission_executor가 별도로 소유한다.
        self.declare_parameter("mission_oscillation_cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("mission_oscillation_reversal_count", 4)
        self.declare_parameter("mission_oscillation_angular_threshold", 0.10)
        self.declare_parameter("mission_oscillation_max_linear_speed", 0.03)
        self.declare_parameter("mission_oscillation_window_s", 6.0)
        self.declare_parameter("mission_recovery_forward_distance", 0.20)
        self.declare_parameter("mission_recovery_forward_speed", 0.08)
        self.declare_parameter("mission_recovery_time_allowance", 5.0)
        self.declare_parameter("mission_recovery_cooldown_s", 5.0)
        self.declare_parameter("mission_recovery_max_retries", 3)
        self.declare_parameter("mission_recovery_sensor_timeout_s", 1.0)
        self.declare_parameter("mission_recovery_scan_half_angle_deg", 25.0)
        self.declare_parameter("mission_recovery_front_extent", 0.12)
        self.declare_parameter("mission_recovery_half_width", 0.095)
        self.declare_parameter("mission_recovery_safety_margin", 0.10)
        self.declare_parameter("mission_recovery_scan_topic", "/scan_raw")
        self.declare_parameter(
            "mission_recovery_local_costmap_topic", "/local_costmap/costmap"
        )
        self.declare_parameter(
            "mission_approach_global_costmap_topic", "/global_costmap/costmap"
        )

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
        self.keepout_suppress_confirm_timeout = (
            self.get_parameter("keepout_suppress_confirm_timeout").value
        )
        self.person_approach_distance = self.get_parameter(
            "person_approach_distance"
        ).value
        self.fire_approach_distance = self.get_parameter(
            "fire_approach_distance"
        ).value
        self.approach_clearance_radius = self.get_parameter(
            "approach_clearance_radius"
        ).value
        self.costmap_occupied_threshold = self.get_parameter(
            "costmap_occupied_threshold"
        ).value

        self.mission_oscillation_reversal_count = self.get_parameter(
            "mission_oscillation_reversal_count"
        ).value
        self.mission_oscillation_angular_threshold = self.get_parameter(
            "mission_oscillation_angular_threshold"
        ).value
        self.mission_oscillation_max_linear_speed = self.get_parameter(
            "mission_oscillation_max_linear_speed"
        ).value
        self.mission_oscillation_window_s = self.get_parameter(
            "mission_oscillation_window_s"
        ).value
        self.mission_recovery_forward_distance = self.get_parameter(
            "mission_recovery_forward_distance"
        ).value
        self.mission_recovery_forward_speed = self.get_parameter(
            "mission_recovery_forward_speed"
        ).value
        self.mission_recovery_time_allowance = self.get_parameter(
            "mission_recovery_time_allowance"
        ).value
        self.mission_recovery_cooldown_s = self.get_parameter(
            "mission_recovery_cooldown_s"
        ).value
        self.mission_recovery_max_retries = self.get_parameter(
            "mission_recovery_max_retries"
        ).value
        self.mission_recovery_sensor_timeout_s = self.get_parameter(
            "mission_recovery_sensor_timeout_s"
        ).value
        self.mission_recovery_scan_half_angle = math.radians(
            self.get_parameter("mission_recovery_scan_half_angle_deg").value
        )
        self.mission_recovery_front_extent = self.get_parameter(
            "mission_recovery_front_extent"
        ).value
        self.mission_recovery_half_width = self.get_parameter(
            "mission_recovery_half_width"
        ).value
        self.mission_recovery_safety_margin = self.get_parameter(
            "mission_recovery_safety_margin"
        ).value

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
        # mask 임시 해제 요청을 보내고 fire_keepout_node 의 확인
        # (circles 갱신)을 기다리는 동안만 채워진다: (x, y, radius, distance)
        self._escape_pending = None
        self._escape_deadline = None
        self._escape_x = 0.0
        self._escape_y = 0.0
        self._escape_distance = 0.0

        # -----------------------------
        # 불 방향 정렬 (fire goal 도착 후, 진압 호출 전)
        # -----------------------------
        self._facing_fire = False
        self._face_pending = None  # 확인 대기 중일 때만 채워짐: (x, y)
        self._face_deadline = None
        self._face_x = 0.0
        self._face_y = 0.0

        self._spin_client = ActionClient(self, Spin, "spin")
        self._drive_client = ActionClient(self, DriveOnHeading, "drive_on_heading")
        self._suppress_pub = self.create_publisher(
            String, "/fire_keepout_suppress", 10
        )

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
        self._nav_source_target_xy = None  # 접근점 계산 전 사람/불 원본 좌표
        # goal 을 보낼 때마다 증가 — 취소된 이전 goal 의 뒤늦은 결과
        # 콜백이 지금 goal 의 상태를 덮어쓰지 못하게 막는 용도.
        self._nav_goal_token = 0

        # 사람/불 mission goal의 회전 진동 및 안전 전진 recovery 상태.
        self._mission_last_angular_sign = 0
        self._mission_last_angular_command_at_ns = None
        self._mission_reversal_times_ns = deque()
        self._mission_recovering = False
        self._mission_recovery_token = 0
        self._mission_recovery_drive_handle = None
        self._mission_recovery_retry_count = 0
        self._mission_last_recovery_at_ns = None
        self._mission_recovery_exhausted = False
        self._mission_target_key = None

        self._latest_scan = None
        self._latest_scan_at_ns = None
        self._latest_local_costmap = None
        self._latest_local_costmap_at_ns = None
        self._latest_global_costmap = None

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

        self.create_subscription(
            Twist,
            self.get_parameter("mission_oscillation_cmd_vel_topic").value,
            self._mission_cmd_vel_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("mission_recovery_scan_topic").value,
            self._mission_scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("mission_recovery_local_costmap_topic").value,
            self._mission_local_costmap_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("mission_approach_global_costmap_topic").value,
            self._mission_global_costmap_callback,
            qos_profile_sensor_data,
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

        self._event_logger.info(
            "Mission oscillation recovery: "
            f"reversals={self.mission_oscillation_reversal_count}/"
            f"{self.mission_oscillation_window_s:.1f}s, "
            f"forward={self.mission_recovery_forward_distance:.2f}m, "
            f"retries={self.mission_recovery_max_retries}, "
            "safety=local_costmap+scan"
        )

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

        if msg.data not in (
            StateManager.PERSON_DETECTED,
            StateManager.FIRE_DETECTED,
        ):
            self._abort_mission_recovery()
            self._reset_mission_oscillation_detector()
        elif msg.data != self.state:
            # 같은 좌표에서 person/fire 상태가 바뀌어도 별개의 mission이다.
            self._abort_mission_recovery()
            self._mission_recovery_retry_count = 0
            self._mission_last_recovery_at_ns = None
            self._mission_recovery_exhausted = False
            self._reset_mission_oscillation_detector()

        self.state = msg.data

    def target_callback(self, msg):
        target_key = (
            round(msg.pose.position.x, 3),
            round(msg.pose.position.y, 3),
        )
        if target_key != self._mission_target_key:
            self._abort_mission_recovery()
            self._mission_target_key = target_key
            self._mission_recovery_retry_count = 0
            self._mission_last_recovery_at_ns = None
            self._mission_recovery_exhausted = False
            self._reset_mission_oscillation_detector()
        self.current_target = msg

    def _mission_scan_callback(self, msg):
        self._latest_scan = msg
        self._latest_scan_at_ns = self.get_clock().now().nanoseconds

    def _mission_local_costmap_callback(self, msg):
        self._latest_local_costmap = msg
        self._latest_local_costmap_at_ns = self.get_clock().now().nanoseconds

    def _mission_global_costmap_callback(self, msg):
        self._latest_global_costmap = msg

    def _keepout_circles_callback(self, msg):

        try:
            circles = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f"Invalid fire_keepout_circles JSON: {e}")
            return

        self._keepout_circles = [
            (c["x"], c["y"], c["radius"]) for c in circles
        ]

        if self._escape_pending is not None:
            x, y, radius, distance = self._escape_pending

            still_present = any(
                cx == x and cy == y for cx, cy, _ in self._keepout_circles
            )

            if not still_present:
                self._escape_pending = None
                self._proceed_escape(x, y, radius, distance)

        if self._face_pending is not None:
            x, y = self._face_pending

            still_present = any(
                cx == x and cy == y for cx, cy, _ in self._keepout_circles
            )

            if not still_present:
                self._face_pending = None
                self._proceed_face_fire(x, y)

    # =========================================================
    # Timer / State machine
    # =========================================================

    def timer_callback(self):

        if self._mission_recovering:
            return

        if self._escaping:
            if (
                self._escape_pending is not None
                and self.get_clock().now() >= self._escape_deadline
            ):
                x, y, _, _ = self._escape_pending
                self.get_logger().warn(
                    "keepout mask 임시 해제 확인 시간초과 — 이탈 포기하고 재시도"
                )
                self._escape_pending = None
                self._finish_escape(x, y)
            return

        if self._facing_fire:
            if (
                self._face_pending is not None
                and self.get_clock().now() >= self._face_deadline
            ):
                x, y = self._face_pending
                self.get_logger().warn(
                    "불 방향 정렬용 keepout mask 임시 해제 확인 시간초과 — "
                    "정렬 포기하고 진압 진행"
                )
                self._face_pending = None
                self._finish_face_fire(x, y)
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

            if (
                self.state in (
                    StateManager.PERSON_DETECTED,
                    StateManager.FIRE_DETECTED,
                )
                and self._mission_recovery_exhausted
            ):
                return

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

    def _approach_distance_for_state(self):
        if self.state == StateManager.PERSON_DETECTED:
            return self.person_approach_distance
        return self.fire_approach_distance

    def _outside_all_keepouts(self, x, y):
        for keepout_x, keepout_y, radius in self._keepout_circles:
            if math.hypot(x - keepout_x, y - keepout_y) < (
                radius + self.approach_clearance_radius
            ):
                return False
        return True

    def _global_costmap_point_is_clear(self, x, y):
        grid = self._latest_global_costmap
        if grid is None:
            # keepout circles는 별도로 검사한다. costmap 첫 수신 전에도
            # mission 전체를 멈추지는 않고 Nav2의 planner 검사를 맡긴다.
            return True

        return occupancy_disk_is_clear(
            grid.data,
            grid.info.width,
            grid.info.height,
            grid.info.resolution,
            grid.info.origin.position.x,
            grid.info.origin.position.y,
            x,
            y,
            self.approach_clearance_radius,
            self.costmap_occupied_threshold,
            True,
        )

    def _make_approach_goal(self, target):
        pose = self._robot_pose_yaw()
        if pose is None:
            self.get_logger().warn(
                "접근 목표 계산용 map->base TF를 찾을 수 없음",
                throttle_duration_sec=2.0,
            )
            return None

        robot_x, robot_y, _ = pose
        target_x = target.pose.position.x
        target_y = target.pose.position.y
        distance = self._approach_distance_for_state()

        target_circle = self._find_keepout_circle(target_x, target_y)
        if target_circle is not None:
            distance = max(
                distance,
                target_circle[2] + self.approach_clearance_radius + 0.05,
            )

        selected = None
        for candidate_x, candidate_y in approach_candidates(
            robot_x, robot_y, target_x, target_y, distance
        ):
            if not self._outside_all_keepouts(candidate_x, candidate_y):
                continue
            if not self._global_costmap_point_is_clear(candidate_x, candidate_y):
                continue
            selected = (candidate_x, candidate_y)
            break

        if selected is None:
            self.get_logger().warn(
                f"사람/불 ({target_x:.2f}, {target_y:.2f}) 주변에서 "
                "keepout 바깥의 안전한 접근 목표를 찾지 못함",
                throttle_duration_sec=2.0,
            )
            return None

        candidate_x, candidate_y = selected
        face_yaw = math.atan2(target_y - candidate_y, target_x - candidate_x)

        goal = PoseStamped()
        goal.header.frame_id = target.header.frame_id or self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = candidate_x
        goal.pose.position.y = candidate_y
        goal.pose.position.z = target.pose.position.z
        goal.pose.orientation.z = math.sin(face_yaw * 0.5)
        goal.pose.orientation.w = math.cos(face_yaw * 0.5)
        return goal

    def _send_nav_goal(self, pose_stamped):

        if pose_stamped is None:
            return

        source_target_xy = (
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
        )

        if source_target_xy == self._nav_source_target_xy:
            # 같은 검출 대상의 접근 goal이 전송/진행 중이면
            # 재계산하지 않는다.
            return

        nav_pose = pose_stamped
        if self.state in (
            StateManager.PERSON_DETECTED,
            StateManager.FIRE_DETECTED,
        ):
            nav_pose = self._make_approach_goal(pose_stamped)
            if nav_pose is None:
                return

        target_xy = (
            nav_pose.pose.position.x,
            nav_pose.pose.position.y,
        )

        if not self._nav_client.server_is_ready():
            self.get_logger().warn(
                "navigate_to_pose 액션 서버가 아직 준비되지 않음",
                throttle_duration_sec=2.0,
            )
            return

        # 목적지가 바뀌었으니 진행 중이던 goal 은 취소하고 새로 보낸다
        self._cancel_nav_goal()

        self._nav_goal_xy = target_xy
        self._nav_source_target_xy = source_target_xy
        self._nav_goal_token += 1
        token = self._nav_goal_token

        goal = NavigateToPose.Goal()
        goal.pose = nav_pose

        self._event_logger.info(
            f"Nav2 접근 goal 설정: target=({source_target_xy[0]:.2f}, "
            f"{source_target_xy[1]:.2f}) -> approach=({target_xy[0]:.2f}, "
            f"{target_xy[1]:.2f})"
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
        self._nav_source_target_xy = None
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
            self._nav_source_target_xy = None
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
                # 도착만으론 완료가 아니다 — 불 방향으로 헤딩을 맞춘 뒤
                # 진압 노드를 불러서 그 응답이 와야 완료 처리한다.
                fire_xy = self._nav_source_target_xy or self._nav_goal_xy
                self._start_face_fire(*fire_xy)
            else:
                # RETURNING_TO_CHARGE/EXPLORING 은 active_target 이 없어서
                # 호출해도 무시되고(no-op), person 은 이걸로 완료 처리된다.
                self.notify_target_complete()
        else:
            self._nav_goal_xy = None
            self._nav_source_target_xy = None
            self.get_logger().warn(
                f"Nav2 goal 이 실패함 — 도달 불가로 보고 다음 목적지로 "
                f"넘어감 (status={status})"
            )
            self.notify_target_complete(status=StateManager.TARGET_STATUS_UNREACHABLE)

    # =========================================================
    # 사람/불 접근 중 회전 진동 recovery
    # =========================================================

    def _reset_mission_oscillation_detector(self):
        self._mission_last_angular_sign = 0
        self._mission_last_angular_command_at_ns = None
        self._mission_reversal_times_ns.clear()

    def _mission_cmd_vel_callback(self, msg):
        if (
            self.state not in (
                StateManager.PERSON_DETECTED,
                StateManager.FIRE_DETECTED,
            )
            or self._nav_goal_handle is None
            or self._mission_recovering
            or self._escaping
            or self._facing_fire
            or self._mission_recovery_exhausted
        ):
            self._reset_mission_oscillation_detector()
            return

        linear_speed = math.hypot(msg.linear.x, msg.linear.y)
        if linear_speed > self.mission_oscillation_max_linear_speed:
            self._reset_mission_oscillation_detector()
            return

        if abs(msg.angular.z) < self.mission_oscillation_angular_threshold:
            return

        now_ns = self.get_clock().now().nanoseconds
        window_ns = int(self.mission_oscillation_window_s * 1e9)
        cooldown_ns = int(self.mission_recovery_cooldown_s * 1e9)

        if (
            self._mission_last_recovery_at_ns is not None
            and now_ns - self._mission_last_recovery_at_ns < cooldown_ns
        ):
            self._reset_mission_oscillation_detector()
            return

        if (
            self._mission_last_angular_command_at_ns is not None
            and now_ns - self._mission_last_angular_command_at_ns > window_ns
        ):
            self._reset_mission_oscillation_detector()

        self._mission_last_angular_command_at_ns = now_ns
        while (
            self._mission_reversal_times_ns
            and now_ns - self._mission_reversal_times_ns[0] > window_ns
        ):
            self._mission_reversal_times_ns.popleft()

        sign = 1 if msg.angular.z > 0.0 else -1
        if self._mission_last_angular_sign == 0:
            self._mission_last_angular_sign = sign
            return
        if sign == self._mission_last_angular_sign:
            return

        self._mission_last_angular_sign = sign
        self._mission_reversal_times_ns.append(now_ns)
        if len(self._mission_reversal_times_ns) < (
            self.mission_oscillation_reversal_count
        ):
            return

        self._reset_mission_oscillation_detector()
        self._start_mission_oscillation_recovery()

    def _start_mission_oscillation_recovery(self):
        if self._mission_recovering or self._nav_goal_handle is None:
            return

        if self._mission_recovery_retry_count >= self.mission_recovery_max_retries:
            self._exhaust_mission_recovery("회전 진동 recovery 재시도 한도 초과")
            return

        self._mission_recovering = True
        self._mission_recovery_retry_count += 1
        self._mission_recovery_token += 1
        token = self._mission_recovery_token

        goal_handle = self._nav_goal_handle
        self._nav_goal_handle = None
        self._nav_goal_xy = None
        self._nav_source_target_xy = None
        self._nav_goal_token += 1

        self._event_logger.warn(
            "사람/불 접근 회전 진동 감지 — Nav2 goal 취소 후 안전 전진 검사 "
            f"({self._mission_recovery_retry_count}/"
            f"{self.mission_recovery_max_retries})"
        )

        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            functools.partial(
                self._mission_recovery_cancel_done,
                token=token,
                goal_handle=goal_handle,
            )
        )

    def _mission_recovery_cancel_done(self, future, token, goal_handle):
        if token != self._mission_recovery_token or not self._mission_recovering:
            return

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn(f"회전 진동 goal 취소 실패: {exc}")
            self._finish_mission_recovery(False)
            return

        if not response.goals_canceling:
            self.get_logger().warn("회전 진동 goal 취소가 수락되지 않음")
            self._finish_mission_recovery(False)
            return

        # cancel 응답은 요청 수락일 뿐 controller 정지를 뜻하지 않는다.
        # NavigateToPose의 최종 CANCELED 결과 뒤에만 behavior를 시작한다.
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            functools.partial(self._mission_recovery_nav_stopped, token=token)
        )

    def _mission_recovery_nav_stopped(self, future, token):
        if token != self._mission_recovery_token or not self._mission_recovering:
            return

        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().warn(f"취소된 Nav2 goal 결과 수신 실패: {exc}")
            self._finish_mission_recovery(False)
            return

        if status != GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                "회전 진동 취소 중 Nav2 goal이 다른 상태로 종료됨"
                f"(status={status})"
            )
            self._finish_mission_recovery(False)
            return

        safe, reason = self._mission_forward_is_safe()
        if not safe:
            self.get_logger().warn(f"회전 진동 recovery 안전 전진 거부: {reason}")
            self._finish_mission_recovery(False)
            return

        if not self._drive_client.server_is_ready():
            self.get_logger().warn("recovery용 drive_on_heading 서버가 준비되지 않음")
            self._finish_mission_recovery(False)
            return

        goal = DriveOnHeading.Goal()
        goal.target = Point(
            x=self.mission_recovery_forward_distance,
            y=0.0,
            z=0.0,
        )
        goal.speed = self.mission_recovery_forward_speed
        goal.time_allowance = ActionDuration(
            sec=max(1, int(math.ceil(self.mission_recovery_time_allowance)))
        )

        send_future = self._drive_client.send_goal_async(goal)
        send_future.add_done_callback(
            functools.partial(self._mission_recovery_drive_response, token=token)
        )

    def _mission_forward_is_safe(self):
        now_ns = self.get_clock().now().nanoseconds
        timeout_ns = int(self.mission_recovery_sensor_timeout_s * 1e9)

        if self._latest_local_costmap is None:
            return False, "local costmap 미수신"
        if (
            self._latest_local_costmap_at_ns is None
            or now_ns - self._latest_local_costmap_at_ns > timeout_ns
        ):
            return False, "local costmap 데이터가 오래됨"
        if self._latest_scan is None:
            return False, "LiDAR scan 미수신"
        if (
            self._latest_scan_at_ns is None
            or now_ns - self._latest_scan_at_ns > timeout_ns
        ):
            return False, "LiDAR scan 데이터가 오래됨"

        grid = self._latest_local_costmap
        grid_frame = grid.header.frame_id or "odom"
        pose = self._robot_pose_in_frame(grid_frame)
        if pose is None:
            return False, f"{grid_frame}->base TF 조회 실패"

        robot_x, robot_y, robot_yaw = pose
        if not forward_corridor_is_clear(
            grid.data,
            grid.info.width,
            grid.info.height,
            grid.info.resolution,
            grid.info.origin.position.x,
            grid.info.origin.position.y,
            robot_x,
            robot_y,
            robot_yaw,
            self.mission_recovery_forward_distance,
            self.mission_recovery_front_extent,
            self.mission_recovery_half_width,
            self.mission_recovery_safety_margin,
            self.costmap_occupied_threshold,
            True,
        ):
            return False, "local costmap 전방 corridor가 점유됨"

        scan = self._latest_scan
        required_scan_range = (
            self.mission_recovery_forward_distance
            + self.mission_recovery_front_extent
            + self.mission_recovery_safety_margin
        )
        if not front_scan_is_clear(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            scan.range_min,
            scan.range_max,
            self.mission_recovery_scan_half_angle,
            required_scan_range,
        ):
            return False, "LiDAR 정면 안전거리 부족 또는 유효 ray 없음"

        return True, "costmap 및 LiDAR 안전"

    def _mission_recovery_drive_response(self, future, token):
        if token != self._mission_recovery_token or not self._mission_recovering:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f"recovery 전진 goal 전송 실패: {exc}")
            self._finish_mission_recovery(False)
            return

        if not goal_handle.accepted:
            self.get_logger().warn("recovery용 drive_on_heading goal이 거부됨")
            self._finish_mission_recovery(False)
            return

        self._mission_recovery_drive_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            functools.partial(self._mission_recovery_drive_result, token=token)
        )

    def _mission_recovery_drive_result(self, future, token):
        if token != self._mission_recovery_token or not self._mission_recovering:
            return

        self._mission_recovery_drive_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self.get_logger().warn(f"recovery 전진 결과 수신 실패: {exc}")
            self._finish_mission_recovery(False)
            return

        self._finish_mission_recovery(status == GoalStatus.STATUS_SUCCEEDED)

    def _finish_mission_recovery(self, advanced):
        self._mission_recovering = False
        self._mission_recovery_drive_handle = None
        self._mission_last_recovery_at_ns = self.get_clock().now().nanoseconds
        self._reset_mission_oscillation_detector()

        if advanced:
            self._event_logger.info(
                "회전 진동 recovery 안전 전진 완료 — 접근 목표를 다시 계산함"
            )
            return

        self.get_logger().warn(
            "회전 진동 recovery가 전진하지 못함 — cooldown 후 "
            "같은 대상을 재시도"
        )
        if self._mission_recovery_retry_count >= self.mission_recovery_max_retries:
            self._exhaust_mission_recovery("안전 전진 recovery 반복 실패")

    def _abort_mission_recovery(self):
        if not self._mission_recovering:
            return

        self._mission_recovery_token += 1
        if self._mission_recovery_drive_handle is not None:
            self._mission_recovery_drive_handle.cancel_goal_async()
        self._mission_recovery_drive_handle = None
        self._mission_recovering = False
        self._reset_mission_oscillation_detector()

    def _exhaust_mission_recovery(self, reason):
        if self._mission_recovery_exhausted:
            return

        self._mission_recovering = False
        self._mission_recovery_exhausted = True
        self._cancel_nav_goal()
        self.get_logger().error(
            f"{reason} — 현재 사람/불 목표를 도달 불가로 처리함"
        )
        self.notify_target_complete(status=StateManager.TARGET_STATUS_UNREACHABLE)

    # =========================================================
    # Keepout 이탈 — 주행 중 로봇 발판이 keepout 원과 겹치면, 그 원의
    # 중심에서 로봇 쪽으로 향하는 방향으로 회전(Spin) 후 전진
    # (DriveOnHeading) 해서 빠져나간다. 방향을 우리가 직접 계산해서
    # 주는 이유는 nav2 기본 recovery(Spin/BackUp) 는 이게 우리가 만든
    # 원이라는 걸 몰라서 임의 방향으로 시도하기 때문 — 잘못된 방향으로
    # 후퇴하면 안 풀릴 수 있다.
    #
    # Spin 은 회전 스윕 전체를 로컬 costmap 기준으로 충돌검사한다
    # (nav2_behaviors::Spin::isCollisionFree). 로봇 발판이 이미 keepout
    # 에 걸친 상태에서는 어느 방향으로 돌아도 그 lethal 셀을 스치게 돼서
    # 매번 "Collision Ahead"로 중간에 멈추고 무한 재시도에 빠진다 — 실제
    # 하드웨어에서 확인된 문제. 그래서 회전/전진 전에 fire_keepout_node
    # 에게 그 원 하나만 mask 에서 잠깐 빼달라고 요청하고, circles 토픽에
    # 반영된 걸 확인한 뒤에야 Spin 을 시작한다(실제 라이다 장애물은
    # obstacle_layer 그대로라 안전은 유지됨).
    # =========================================================

    def _robot_pose_in_frame(self, frame):

        try:
            transform = self.tf_buffer.lookup_transform(
                frame,
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

    def _robot_pose_yaw(self):
        return self._robot_pose_in_frame(self.map_frame)

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

    def _publish_suppress(self, x, y, suppress):

        msg = String()
        msg.data = json.dumps({"x": x, "y": y, "suppress": suppress})
        self._suppress_pub.publish(msg)

    def _start_keepout_escape(self, overlap):

        x, y, radius, distance = overlap

        self._escaping = True
        self._cancel_nav_goal()

        self._escape_pending = (x, y, radius, distance)
        self._escape_deadline = (
            self.get_clock().now()
            + Duration(seconds=self.keepout_suppress_confirm_timeout)
        )

        self._event_logger.info(
            f"keepout 이탈 시작: ({x:.2f}, {y:.2f}) r={radius:.2f}m 안으로 "
            f"{distance:.2f}m — mask 임시 해제 요청"
        )

        self._publish_suppress(x, y, True)

    def _finish_escape(self, x, y):
        """성공/실패/시간초과 상관없이 이탈 시퀀스를 끝낸다 — 잠깐 뺐던
        keepout 을 되돌리고 다음 tick 부터 정상 주행을 재개한다."""

        self._publish_suppress(x, y, False)
        self._escaping = False

    def _proceed_escape(self, x, y, radius, distance):
        """fire_keepout_node 가 mask 에서 해당 원을 뺀 걸 확인한 뒤에만
        호출된다 — 이제 Spin 이 그 자리를 충돌로 안 본다."""

        pose = self._robot_pose_yaw()

        if pose is None:
            self._finish_escape(x, y)
            return

        if not self._spin_client.server_is_ready():
            self.get_logger().warn(
                "spin 액션 서버가 아직 준비되지 않음 — keepout 이탈 중단"
            )
            self._finish_escape(x, y)
            return

        px, py, yaw = pose

        self._escape_x, self._escape_y = x, y

        escape_angle = math.atan2(py - y, px - x)
        relative_yaw = math.atan2(
            math.sin(escape_angle - yaw), math.cos(escape_angle - yaw)
        )
        # 여유(margin) + 약간의 버퍼(0.05m)만큼 더 벌어질 때까지 전진.
        self._escape_distance = (
            radius + self.keepout_escape_margin + 0.05 - distance
        )

        self._event_logger.info(
            f"keepout 이탈: ({x:.2f}, {y:.2f}) mask 해제 확인 — "
            f"{math.degrees(relative_yaw):.0f}도 회전 후 "
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
            self._finish_escape(self._escape_x, self._escape_y)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._spin_goal_result)

    def _spin_goal_result(self, future):

        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f"keepout 이탈용 spin 실패(status={status}) — 다음 tick 에 재시도"
            )
            self._finish_escape(self._escape_x, self._escape_y)
            return

        if not self._drive_client.server_is_ready():
            self.get_logger().warn("drive_on_heading 액션 서버가 아직 준비되지 않음")
            self._finish_escape(self._escape_x, self._escape_y)
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
            self._finish_escape(self._escape_x, self._escape_y)
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

        self._finish_escape(self._escape_x, self._escape_y)

    # =========================================================
    # 불 방향 정렬 — fire goal 도착 후 진압 전에, escape 와 같은 mask
    # 임시 해제 시퀀스로 안전하게 불 쪽으로 회전한다.
    # =========================================================

    def _find_keepout_circle(self, x, y, tolerance=0.05):
        """fire_keepout_node 는 좌표를 소수점 2자리로 반올림해서 추적/
        발행하므로, nav goal 의 원본 좌표로 직접 비교하면 절대 안
        맞는다 — 가장 가까운 원을 찾아 그 반올림된 좌표를 대신 쓴다."""

        for cx, cy, radius in self._keepout_circles:
            if math.hypot(cx - x, cy - y) <= tolerance:
                return (cx, cy, radius)

        return None

    def _start_face_fire(self, x, y):

        circle = self._find_keepout_circle(x, y)

        if circle is None:
            # mask 에 이미 없으면(예: 어떤 이유로 등록이 안 됐거나 이미
            # suppress 됨) 정렬을 건너뛰고 바로 진압으로 넘어간다.
            self._call_fire_suppression()
            return

        x, y, _ = circle

        self._facing_fire = True

        self._face_pending = (x, y)
        self._face_deadline = (
            self.get_clock().now()
            + Duration(seconds=self.keepout_suppress_confirm_timeout)
        )

        self._event_logger.info(
            f"불 방향 정렬 시작: ({x:.2f}, {y:.2f}) — mask 임시 해제 요청"
        )

        self._publish_suppress(x, y, True)

    def _finish_face_fire(self, x, y):
        """정렬 성공/실패/시간초과 상관없이 mask 를 되돌리고 진압을
        진행한다 — 헤딩이 안 맞았다고 진압 자체를 포기하진 않는다."""

        self._publish_suppress(x, y, False)
        self._facing_fire = False

        self._call_fire_suppression()

    def _proceed_face_fire(self, x, y):
        """fire_keepout_node 가 mask 에서 해당 원을 뺀 걸 확인한 뒤에만
        호출된다 — 이제 Spin 이 그 자리를 충돌로 안 본다."""

        pose = self._robot_pose_yaw()

        if pose is None:
            self._finish_face_fire(x, y)
            return

        if not self._spin_client.server_is_ready():
            self.get_logger().warn(
                "spin 액션 서버가 아직 준비되지 않음 — 불 방향 정렬 중단"
            )
            self._finish_face_fire(x, y)
            return

        px, py, yaw = pose

        self._face_x, self._face_y = x, y

        face_angle = math.atan2(y - py, x - px)
        relative_yaw = math.atan2(
            math.sin(face_angle - yaw), math.cos(face_angle - yaw)
        )

        self._event_logger.info(
            f"불 방향 정렬: ({x:.2f}, {y:.2f}) mask 해제 확인 — "
            f"{math.degrees(relative_yaw):.0f}도 회전"
        )

        goal = Spin.Goal()
        goal.target_yaw = relative_yaw
        goal.time_allowance = ActionDuration(
            sec=int(self.keepout_escape_time_allowance)
        )

        send_future = self._spin_client.send_goal_async(goal)
        send_future.add_done_callback(self._face_spin_goal_response)

    def _face_spin_goal_response(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("불 방향 정렬용 spin goal 이 거부됨")
            self._finish_face_fire(self._face_x, self._face_y)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._face_spin_goal_result)

    def _face_spin_goal_result(self, future):

        status = future.result().status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f"불 방향 정렬용 spin 실패(status={status}) — 그래도 진압 진행"
            )

        self._finish_face_fire(self._face_x, self._face_y)

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
