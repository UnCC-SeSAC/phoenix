"""Read-only CLI diagnostics for frontier goals, costmaps, and DWB.

The node never sends NavigateToPose or velocity commands.  It optionally asks
Nav2's ComputePathToPose server for a read-only global path check.
"""

import heapq
import math
import re
from collections import deque
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus, GoalStatusArray

from dwb_msgs.msg import LocalPlanEvaluation

from geometry_msgs.msg import PolygonStamped, PoseStamped, Twist

from nav2_msgs.action import ComputePathToPose, NavigateToPose

from nav_msgs.msg import OccupancyGrid, Odometry, Path

from rcl_interfaces.msg import Log

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_action_status_default,
    qos_profile_sensor_data,
)
from rclpy.time import Time

from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformException, TransformListener


Point2 = Tuple[float, float]
Cell = Tuple[int, int]


@dataclass
class GoalRecord:
    """A frontier pose and the source that supplied it."""

    pose: PoseStamped
    source: str
    generation: int


@dataclass
class FootprintMetrics:
    """Rotation-independent robot footprint measurements."""

    inscribed_radius: float
    circumscribed_radius: float
    maximum_span: float
    point_count: int


@dataclass
class RuntimeSample:
    """Compact state retained in the pre-anomaly history window."""

    stamp_ns: int
    goal_generation: int
    robot_pose: Optional[Tuple[float, float, float]]
    goal_distance: Optional[float]
    cmd_linear: float
    cmd_angular: float
    odom_linear: float
    odom_angular: float


@dataclass
class ActiveAnomaly:
    """State used to debounce one persistent anomaly type."""

    first_seen_ns: int
    last_seen_ns: int
    last_report_ns: int
    detail: str


class FrontierDiagnostics(Node):
    """Observe a running exploration stack and print numeric diagnostics."""

    _DISPATCH_RE = re.compile(
        r'dispatch=\(\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\)'
    )
    _WATCHED_LOGGERS = (
        'frontier_explorer',
        'planner_server',
        'controller_server',
        'bt_navigator',
        'behavior_server',
        'global_costmap',
        'local_costmap',
        'slam_toolbox',
        'velocity_smoother',
    )
    _WARN_KEYWORDS = (
        'failed',
        'failure',
        'invalid',
        'empty',
        'no valid',
        'no path',
        'blocked',
        'stuck',
        'timeout',
        'aborted',
        'transform',
        'extrapolation',
        'collision',
        'halt',
    )
    _EXPECTED_CANCEL_KEYWORDS = (
        'preempt',
        'goal replacement',
        'control stop',
        'skipping blocked frontier',
    )

    def __init__(self):
        """Initialize subscriptions, TF, timers, and the planner client."""
        super().__init__('frontier_diagnostics')
        self._declare_parameters()
        self._load_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.map_msg: Optional[OccupancyGrid] = None
        self.global_costmap: Optional[OccupancyGrid] = None
        self.local_costmap: Optional[OccupancyGrid] = None
        self.global_footprint: Optional[PolygonStamped] = None
        self.local_footprint: Optional[PolygonStamped] = None
        self.goal: Optional[GoalRecord] = None
        self.goal_generation = 0
        self.last_analyzed_generation = -1
        self.last_geometry: Dict[str, Tuple] = {}
        self.dwb_message_received = False
        self.path_request_generation = -1
        self.path_request_pending = False
        self.grid_connectivity_failure: Optional[str] = None
        self.start_time_ns = self.get_clock().now().nanoseconds
        self.goal_active = False
        self.goal_status = GoalStatus.STATUS_UNKNOWN
        self.goal_set_ns = 0
        self.goal_status_by_id: Dict[Tuple[int, ...], int] = {}
        self.expected_cancel_until_ns = 0
        self.last_feedback_distance: Optional[float] = None
        self.last_feedback_recoveries = 0
        self.last_feedback_ns = 0
        self.last_odom: Optional[Odometry] = None
        self.last_odom_ns = 0
        self.last_scan_ns = 0
        self.grid_receive_ns: Dict[str, int] = {}
        self.last_cmd: Dict[str, Twist] = {}
        self.last_cmd_receive_ns: Dict[str, int] = {}
        self.path_summaries: Dict[str, Tuple[int, float, int]] = {}
        self.nav2_path_summary = 'not requested'
        self.dwb_summary = 'not received'
        self.dwb_valid_count: Optional[int] = None
        self.passage_details: Dict[str, str] = {}
        self.frontier_details: Dict[str, str] = {}
        self.tf_issue: Optional[str] = None
        self.history: deque[RuntimeSample] = deque()
        self.active_anomalies: Dict[str, ActiveAnomaly] = {}
        self.event_last_report_ns: Dict[str, int] = {}

        self._subscriptions = []
        self._create_subscriptions()

        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            self.compute_path_action,
        )
        self.timer = self.create_timer(
            self.diagnostic_period_s,
            self._diagnostic_tick,
        )

        self.get_logger().info(
            'Frontier diagnostics started: anomaly_only=%s history=%.1fs '
            'cooldown=%.1fs (read-only; no navigation or velocity goal '
            'is sent)' % (
                self.anomaly_only_logging,
                self.history_window_s,
                self.event_cooldown_s,
            )
        )

    def _declare_parameters(self):
        defaults = {
            'map_topic': '/map',
            'global_costmap_topic': '/global_costmap/costmap',
            'local_costmap_topic': '/local_costmap/costmap',
            'selected_frontier_topic': '/explore/selected_frontier',
            'global_footprint_topic': '/global_costmap/published_footprint',
            'local_footprint_topic': '/local_costmap/published_footprint',
            'map_frame': 'map',
            'base_frame': 'base_footprint',
            'map_occupied_threshold': 65,
            'costmap_blocked_threshold': 65,
            'path_blocked_threshold': 99,
            'lethal_cost_threshold': 99,
            'allow_unknown_path': False,
            'frontier_wall_proximity_m': 0.10,
            'wall_search_radius_m': 1.0,
            'robot_length_m': 0.24,
            'robot_width_m': 0.19,
            'footprint_padding_m': 0.01,
            'passage_probe_distance_m': 0.60,
            'diagnostic_period_s': 1.0,
            'compute_path_enabled': True,
            'compute_path_action': '/compute_path_to_pose',
            'planner_id': 'GridBased',
            'max_grid_path_expansions': 250000,
            'dwb_evaluation_topic': '/evaluation',
            'local_plan_topic': '/local_plan',
            'transformed_global_plan_topic': '/transformed_global_plan',
            'controller_cmd_vel_topic': '/cmd_vel_nav',
            'output_cmd_vel_topic': '/cmd_vel',
            'navigate_status_topic': '/navigate_to_pose/_action/status',
            'navigate_feedback_topic': '/navigate_to_pose/_action/feedback',
            'odom_topic': '/odom',
            'scan_topic': '/scan_raw',
            'anomaly_only_logging': True,
            'history_window_s': 10.0,
            'event_cooldown_s': 10.0,
            'startup_grace_period_s': 15.0,
            'no_progress_timeout_s': 8.0,
            'min_goal_progress_m': 0.05,
            'stuck_timeout_s': 3.0,
            'min_pose_progress_m': 0.02,
            'min_yaw_progress_rad': 0.05,
            'linear_cmd_threshold': 0.03,
            'angular_cmd_threshold': 0.10,
            'zero_cmd_timeout_s': 3.0,
            'data_stale_timeout_s': 3.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self):
        string_names = (
            'map_topic',
            'global_costmap_topic',
            'local_costmap_topic',
            'selected_frontier_topic',
            'global_footprint_topic',
            'local_footprint_topic',
            'map_frame',
            'base_frame',
            'compute_path_action',
            'planner_id',
            'dwb_evaluation_topic',
            'local_plan_topic',
            'transformed_global_plan_topic',
            'controller_cmd_vel_topic',
            'output_cmd_vel_topic',
            'navigate_status_topic',
            'navigate_feedback_topic',
            'odom_topic',
            'scan_topic',
        )
        int_names = (
            'map_occupied_threshold',
            'costmap_blocked_threshold',
            'path_blocked_threshold',
            'lethal_cost_threshold',
            'max_grid_path_expansions',
        )
        float_names = (
            'frontier_wall_proximity_m',
            'wall_search_radius_m',
            'robot_length_m',
            'robot_width_m',
            'footprint_padding_m',
            'passage_probe_distance_m',
            'diagnostic_period_s',
            'history_window_s',
            'event_cooldown_s',
            'startup_grace_period_s',
            'no_progress_timeout_s',
            'min_goal_progress_m',
            'stuck_timeout_s',
            'min_pose_progress_m',
            'min_yaw_progress_rad',
            'linear_cmd_threshold',
            'angular_cmd_threshold',
            'zero_cmd_timeout_s',
            'data_stale_timeout_s',
        )
        for name in string_names:
            setattr(self, name, str(self.get_parameter(name).value))
        for name in int_names:
            setattr(self, name, int(self.get_parameter(name).value))
        for name in float_names:
            value = float(self.get_parameter(name).value)
            setattr(self, name, max(0.0, value))
        self.allow_unknown_path = bool(
            self.get_parameter('allow_unknown_path').value
        )
        self.compute_path_enabled = bool(
            self.get_parameter('compute_path_enabled').value
        )
        self.anomaly_only_logging = bool(
            self.get_parameter('anomaly_only_logging').value
        )
        self.diagnostic_period_s = max(0.1, self.diagnostic_period_s)

    @staticmethod
    def _map_qos() -> QoSProfile:
        return QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

    @staticmethod
    def _volatile_qos(depth: int = 10) -> QoSProfile:
        return QoSProfile(
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _sub(self, msg_type, topic, callback, qos):
        self._subscriptions.append(
            self.create_subscription(msg_type, topic, callback, qos)
        )

    def _create_subscriptions(self):
        self._sub(
            OccupancyGrid,
            self.map_topic,
            lambda msg: self._grid_callback('map', msg),
            self._map_qos(),
        )
        self._sub(
            OccupancyGrid,
            self.global_costmap_topic,
            lambda msg: self._grid_callback('global', msg),
            self._volatile_qos(),
        )
        self._sub(
            OccupancyGrid,
            self.local_costmap_topic,
            lambda msg: self._grid_callback('local', msg),
            self._volatile_qos(),
        )
        self._sub(
            PoseStamped,
            self.selected_frontier_topic,
            self._selected_frontier_callback,
            self._volatile_qos(),
        )
        self._sub(
            PolygonStamped,
            self.global_footprint_topic,
            lambda msg: setattr(self, 'global_footprint', msg),
            self._volatile_qos(),
        )
        self._sub(
            PolygonStamped,
            self.local_footprint_topic,
            lambda msg: setattr(self, 'local_footprint', msg),
            self._volatile_qos(),
        )
        self._sub(
            Log,
            '/rosout',
            self._rosout_callback,
            self._volatile_qos(100),
        )
        self._sub(
            LocalPlanEvaluation,
            self.dwb_evaluation_topic,
            self._dwb_callback,
            self._volatile_qos(),
        )
        self._sub(
            Path,
            self.local_plan_topic,
            lambda msg: self._path_callback('DWB local_plan', msg),
            self._volatile_qos(),
        )
        self._sub(
            Path,
            self.transformed_global_plan_topic,
            lambda msg: self._path_callback(
                'DWB transformed_global_plan',
                msg,
            ),
            self._volatile_qos(),
        )
        self._sub(
            Twist,
            self.controller_cmd_vel_topic,
            lambda msg: self._cmd_callback('controller', msg),
            self._volatile_qos(),
        )
        self._sub(
            Twist,
            self.output_cmd_vel_topic,
            lambda msg: self._cmd_callback('smoothed', msg),
            self._volatile_qos(),
        )
        self._sub(
            GoalStatusArray,
            self.navigate_status_topic,
            self._goal_status_callback,
            qos_profile_action_status_default,
        )
        self._sub(
            NavigateToPose.Impl.FeedbackMessage,
            self.navigate_feedback_topic,
            self._navigation_feedback_callback,
            self._volatile_qos(),
        )
        self._sub(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self._sub(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

    def _grid_callback(self, label: str, msg: OccupancyGrid):
        now_ns = self.get_clock().now().nanoseconds
        self.grid_receive_ns[label] = now_ns
        if label == 'map':
            self.map_msg = msg
        elif label == 'global':
            self.global_costmap = msg
        else:
            self.local_costmap = msg

        fixed_geometry = (
            msg.header.frame_id,
            int(msg.info.width),
            int(msg.info.height),
            float(msg.info.resolution),
        )
        geometry = fixed_geometry + (
            round(float(msg.info.origin.position.x), 4),
            round(float(msg.info.origin.position.y), 4),
        )
        # A rolling local costmap changes origin as the robot moves.  That is
        # normal window motion, not a resize.  Map/global origin changes are
        # retained because they indicate SLAM/static-layer geometry changes.
        if label == 'local':
            geometry = fixed_geometry
        previous = self.last_geometry.get(label)
        if previous is not None and previous != geometry:
            self._transient_anomaly(
                f'{label.upper()}_RESIZE',
                'WARN',
                f'{previous} -> {geometry}',
            )
        self.last_geometry[label] = geometry

    def _selected_frontier_callback(self, msg: PoseStamped):
        self._set_goal(msg, 'selected_frontier_topic')

    def _rosout_callback(self, msg: Log):
        text = msg.msg
        lower = text.lower()
        now_ns = self.get_clock().now().nanoseconds

        if msg.name.endswith('frontier_explorer'):
            if any(word in lower for word in self._EXPECTED_CANCEL_KEYWORDS):
                self.expected_cancel_until_ns = now_ns + int(5.0e9)
            if 'sending frontier goal' in lower:
                match = self._DISPATCH_RE.search(text)
                if match is not None:
                    pose = PoseStamped()
                    pose.header.frame_id = self.map_frame
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = float(match.group(1))
                    pose.pose.position.y = float(match.group(2))
                    pose.pose.orientation.w = 1.0
                    self._set_goal(pose, 'frontier_explorer_rosout')

        if msg.name == self.get_name():
            return
        if not any(msg.name.endswith(name) for name in self._WATCHED_LOGGERS):
            return

        detail = f'node={msg.name} log={text}'
        if 'path is empty' in lower or 'empty path' in lower:
            self._transient_anomaly('EMPTY_PATH_LOG', 'ERROR', detail)
            return
        if 'invalid path' in lower or 'no valid path' in lower:
            self._transient_anomaly('INVALID_PATH_LOG', 'ERROR', detail)
            return
        if 'goal failed' in lower or 'goal aborted' in lower:
            self._transient_anomaly('GOAL_FAILED_LOG', 'ERROR', detail)
            return
        if 'halt' in lower:
            if now_ns > self.expected_cancel_until_ns:
                self._transient_anomaly('UNEXPECTED_NODE_HALT', 'WARN', detail)
            return
        if int(msg.level) >= 40:
            self._transient_anomaly('ROS_ERROR', 'ERROR', detail)
            return
        if (
            int(msg.level) >= 30 and
            any(word in lower for word in self._WARN_KEYWORDS)
        ):
            self._transient_anomaly('MATCHED_WARN', 'WARN', detail)

    def _set_goal(self, pose: PoseStamped, source: str):
        frame = pose.header.frame_id or self.map_frame
        pose.header.frame_id = frame
        if self.goal is not None:
            old = self.goal.pose
            if (
                old.header.frame_id == frame and
                math.hypot(
                    old.pose.position.x - pose.pose.position.x,
                    old.pose.position.y - pose.pose.position.y,
                ) < 0.01
            ):
                return
        self.goal_generation += 1
        self.goal = GoalRecord(pose, source, self.goal_generation)
        self.goal_set_ns = self.get_clock().now().nanoseconds
        self.goal_active = True
        self.goal_status = GoalStatus.STATUS_ACCEPTED
        self.path_request_pending = False
        self.grid_connectivity_failure = None
        self.last_feedback_distance = None
        self.history.clear()
        self._clear_condition('FRONTIER_INVALID', 'new frontier received')
        self.get_logger().info(
            '[STATE] new frontier source=%s frame=%s xy=(%.3f, %.3f)' % (
                source,
                frame,
                pose.pose.position.x,
                pose.pose.position.y,
            )
        )

    def _goal_status_callback(self, msg: GoalStatusArray):
        previous_node_status = self.goal_status
        active = False
        newest_status = GoalStatus.STATUS_UNKNOWN
        newest_stamp = (-1, -1)
        for item in msg.status_list:
            goal_id = tuple(
                int(value) for value in item.goal_info.goal_id.uuid
            )
            status = int(item.status)
            previous = self.goal_status_by_id.get(goal_id)
            self.goal_status_by_id[goal_id] = status
            stamp = item.goal_info.stamp
            stamp_key = (int(stamp.sec), int(stamp.nanosec))
            if stamp_key >= newest_stamp:
                newest_stamp = stamp_key
                newest_status = status
            if status in (
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING,
            ):
                active = True
            if previous == status:
                continue
            if status == GoalStatus.STATUS_ABORTED:
                self.goal_status = status
                self.goal_active = False
                self._transient_anomaly(
                    'GOAL_ABORTED',
                    'ERROR',
                    f'navigate_to_pose goal_id={goal_id} status=ABORTED',
                )
            elif status == GoalStatus.STATUS_CANCELED:
                now_ns = self.get_clock().now().nanoseconds
                if now_ns > self.expected_cancel_until_ns:
                    self.goal_status = status
                    self.goal_active = False
                    self._transient_anomaly(
                        'UNEXPECTED_CANCEL',
                        'WARN',
                        f'navigate_to_pose goal_id={goal_id} status=CANCELED',
                    )
        self.goal_active = active
        if newest_status != GoalStatus.STATUS_UNKNOWN:
            self.goal_status = newest_status
        if self.goal_status != previous_node_status:
            self.get_logger().info(
                '[STATE] navigate_to_pose %s -> %s active=%s' % (
                    self._status_name(previous_node_status),
                    self._status_name(self.goal_status),
                    self.goal_active,
                )
            )

    def _navigation_feedback_callback(self, msg):
        self.goal_active = True
        self.goal_status = GoalStatus.STATUS_EXECUTING
        self.last_feedback_distance = float(msg.feedback.distance_remaining)
        self.last_feedback_recoveries = int(msg.feedback.number_of_recoveries)
        self.last_feedback_ns = self.get_clock().now().nanoseconds

    def _odom_callback(self, msg: Odometry):
        self.last_odom = msg
        self.last_odom_ns = self.get_clock().now().nanoseconds

    def _scan_callback(self, _msg: LaserScan):
        self.last_scan_ns = self.get_clock().now().nanoseconds

    def _transient_anomaly(self, key: str, severity: str, detail: str):
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self.event_last_report_ns.get(key, 0)
        if now_ns - last_ns < int(self.event_cooldown_s * 1e9):
            return
        self.event_last_report_ns[key] = now_ns
        self._emit_snapshot('ANOMALY', key, severity, detail, now_ns)

    def _set_condition(
        self,
        key: str,
        present: bool,
        severity: str,
        detail: str,
    ):
        now_ns = self.get_clock().now().nanoseconds
        state = self.active_anomalies.get(key)
        if not present:
            if state is not None:
                duration = (now_ns - state.first_seen_ns) * 1e-9
                self.get_logger().info(
                    '[RECOVERED] type=%s duration=%.2fs detail=%s' % (
                        key,
                        duration,
                        detail,
                    )
                )
                del self.active_anomalies[key]
            return

        if state is None:
            self.active_anomalies[key] = ActiveAnomaly(
                first_seen_ns=now_ns,
                last_seen_ns=now_ns,
                last_report_ns=now_ns,
                detail=detail,
            )
            self._emit_snapshot('ANOMALY', key, severity, detail, now_ns)
            return

        state.last_seen_ns = now_ns
        state.detail = detail
        if now_ns - state.last_report_ns >= int(self.event_cooldown_s * 1e9):
            state.last_report_ns = now_ns
            self._emit_snapshot(
                'ANOMALY_REPEAT',
                key,
                severity,
                detail,
                now_ns,
            )

    def _clear_condition(self, key: str, detail: str):
        self._set_condition(key, False, 'INFO', detail)

    @staticmethod
    def _status_name(status: int) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, str(status))

    @staticmethod
    def _twist_values(msg: Optional[Twist]) -> Tuple[float, float]:
        if msg is None:
            return 0.0, 0.0
        linear = math.hypot(float(msg.linear.x), float(msg.linear.y))
        return linear, abs(float(msg.angular.z))

    def _message_age(self, stamp_ns: int, now_ns: int) -> str:
        if stamp_ns <= 0:
            return 'never'
        return f'{max(0.0, (now_ns - stamp_ns) * 1e-9):.2f}s'

    def _history_summary(self) -> str:
        if not self.history:
            return 'history=empty'
        first = self.history[0]
        last = self.history[-1]
        duration = (last.stamp_ns - first.stamp_ns) * 1e-9
        pose_delta = 'unavailable'
        yaw_delta = 'unavailable'
        if first.robot_pose is not None and last.robot_pose is not None:
            pose_delta = '%.3fm' % math.hypot(
                last.robot_pose[0] - first.robot_pose[0],
                last.robot_pose[1] - first.robot_pose[1],
            )
            yaw_delta = '%.3frad' % abs(
                math.atan2(
                    math.sin(last.robot_pose[2] - first.robot_pose[2]),
                    math.cos(last.robot_pose[2] - first.robot_pose[2]),
                )
            )
        goal_progress = 'unavailable'
        if first.goal_distance is not None and last.goal_distance is not None:
            goal_progress = '%.3fm' % (
                first.goal_distance - last.goal_distance
            )
        return (
            'window=%.2fs samples=%d pose_delta=%s yaw_delta=%s '
            'goal_progress=%s' % (
                duration,
                len(self.history),
                pose_delta,
                yaw_delta,
                goal_progress,
            )
        )

    def _emit_snapshot(
        self,
        tag: str,
        key: str,
        severity: str,
        detail: str,
        now_ns: int,
    ):
        goal_text = 'none'
        if self.goal is not None:
            pose = self.goal.pose.pose.position
            goal_text = '(%.3f,%.3f) frame=%s source=%s' % (
                pose.x,
                pose.y,
                self.goal.pose.header.frame_id,
                self.goal.source,
            )
        robot = self._robot_pose(self.map_frame)
        robot_text = 'TF unavailable' if robot is None else (
            '(%.3f,%.3f,%.1fdeg)' % (
                robot[0],
                robot[1],
                math.degrees(robot[2]),
            )
        )
        controller_cmd = self.last_cmd.get('controller')
        smoothed_cmd = self.last_cmd.get('smoothed')
        controller_values = self._twist_values(controller_cmd)
        smoothed_values = self._twist_values(smoothed_cmd)
        odom_linear = 0.0
        odom_angular = 0.0
        if self.last_odom is not None:
            twist = self.last_odom.twist.twist
            odom_linear = math.hypot(twist.linear.x, twist.linear.y)
            odom_angular = abs(float(twist.angular.z))
        metrics = self._active_footprint_metrics()
        path_text = '; '.join(
            '%s poses=%d length=%.3fm age=%s' % (
                name,
                value[0],
                value[1],
                self._message_age(value[2], now_ns),
            )
            for name, value in sorted(self.path_summaries.items())
        ) or 'not received'
        frontier_text = '; '.join(
            f'{name}={value}'
            for name, value in sorted(self.frontier_details.items())
        ) or 'not evaluated'
        passage_text = '; '.join(
            f'{name}={value}'
            for name, value in sorted(self.passage_details.items())
        ) or 'not evaluated'
        snapshot = (
            '[%s] type=%s severity=%s\n'
            '  trigger=%s\n'
            '  goal_status=%s active=%s goal=%s\n'
            '  robot=%s %s\n'
            '  feedback_distance=%s recoveries=%d feedback_age=%s\n'
            '  cmd_controller=(linear=%.3f,angular=%.3f) '
            'cmd_smoothed=(linear=%.3f,angular=%.3f)\n'
            '  odom=(linear=%.3f,angular=%.3f) odom_age=%s scan_age=%s\n'
            '  data_age=(map=%s global=%s local=%s)\n'
            '  footprint=(inscribed=%.3f circumscribed=%.3f span=%.3f)\n'
            '  frontier_checks=%s\n'
            '  robot_passage=%s\n'
            '  grid_path=%s nav2_path=%s\n'
            '  local_paths=%s\n'
            '  dwb=%s' % (
                tag,
                key,
                severity,
                detail,
                self._status_name(self.goal_status),
                self.goal_active,
                goal_text,
                robot_text,
                self._history_summary(),
                self.last_feedback_distance,
                self.last_feedback_recoveries,
                self._message_age(self.last_feedback_ns, now_ns),
                controller_values[0],
                controller_values[1],
                smoothed_values[0],
                smoothed_values[1],
                odom_linear,
                odom_angular,
                self._message_age(self.last_odom_ns, now_ns),
                self._message_age(self.last_scan_ns, now_ns),
                self._message_age(self.grid_receive_ns.get('map', 0), now_ns),
                self._message_age(
                    self.grid_receive_ns.get('global', 0),
                    now_ns,
                ),
                self._message_age(
                    self.grid_receive_ns.get('local', 0),
                    now_ns,
                ),
                metrics.inscribed_radius,
                metrics.circumscribed_radius,
                metrics.maximum_span,
                frontier_text,
                passage_text,
                self.grid_connectivity_failure or 'connected/not checked',
                self.nav2_path_summary,
                path_text,
                self.dwb_summary,
            )
        )
        if severity == 'ERROR':
            self.get_logger().error(snapshot)
        else:
            self.get_logger().warn(snapshot)

    @staticmethod
    def _yaw(quaternion) -> float:
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z +
            quaternion.x * quaternion.y
        )
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y +
            quaternion.z * quaternion.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    def _transform_point(
        self,
        point: Point2,
        source_frame: str,
        target_frame: str,
    ) -> Optional[Point2]:
        if source_frame == target_frame:
            return point
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.15),
            ).transform
        except TransformException as exc:
            self.tf_issue = f'{source_frame} -> {target_frame}: {exc}'
            return None
        self.tf_issue = None
        yaw = self._yaw(transform.rotation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        x, y = point
        return (
            transform.translation.x + cos_yaw * x - sin_yaw * y,
            transform.translation.y + sin_yaw * x + cos_yaw * y,
        )

    def _robot_pose(self, frame: str) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.15),
            ).transform
        except TransformException as exc:
            self.tf_issue = f'{frame} <- {self.base_frame}: {exc}'
            return None
        self.tf_issue = None
        return (
            float(transform.translation.x),
            float(transform.translation.y),
            self._yaw(transform.rotation),
        )

    @staticmethod
    def _world_to_cell(grid: OccupancyGrid, point: Point2) -> Optional[Cell]:
        resolution = float(grid.info.resolution)
        if resolution <= 0.0:
            return None
        origin = grid.info.origin
        yaw = FrontierDiagnostics._yaw(origin.orientation)
        dx = point[0] - float(origin.position.x)
        dy = point[1] - float(origin.position.y)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        mx = int(math.floor(local_x / resolution))
        my = int(math.floor(local_y / resolution))
        if (
            mx < 0 or my < 0 or
            mx >= int(grid.info.width) or
            my >= int(grid.info.height)
        ):
            return None
        return mx, my

    @staticmethod
    def _cell_to_world(grid: OccupancyGrid, cell: Cell) -> Point2:
        resolution = float(grid.info.resolution)
        local_x = (cell[0] + 0.5) * resolution
        local_y = (cell[1] + 0.5) * resolution
        origin = grid.info.origin
        yaw = FrontierDiagnostics._yaw(origin.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            float(origin.position.x) + cos_yaw * local_x - sin_yaw * local_y,
            float(origin.position.y) + sin_yaw * local_x + cos_yaw * local_y,
        )

    @staticmethod
    def _cell_value(grid: OccupancyGrid, cell: Cell) -> int:
        index = cell[1] * int(grid.info.width) + cell[0]
        return int(grid.data[index])

    def _grid_point(
        self,
        point: Point2,
        source_frame: str,
        grid: OccupancyGrid,
    ) -> Optional[Point2]:
        target_frame = grid.header.frame_id or self.map_frame
        return self._transform_point(point, source_frame, target_frame)

    def _nearest_occupied(
        self,
        grid: OccupancyGrid,
        cell: Cell,
        threshold: int,
        search_radius_m: float,
    ) -> Optional[float]:
        resolution = float(grid.info.resolution)
        radius = max(1, int(math.ceil(search_radius_m / resolution)))
        width = int(grid.info.width)
        height = int(grid.info.height)
        nearest_sq: Optional[int] = None
        y_start = max(0, cell[1] - radius)
        y_stop = min(height, cell[1] + radius + 1)
        x_start = max(0, cell[0] - radius)
        x_stop = min(width, cell[0] + radius + 1)
        for my in range(y_start, y_stop):
            for mx in range(x_start, x_stop):
                value = int(grid.data[my * width + mx])
                if value < threshold:
                    continue
                distance_sq = (mx - cell[0]) ** 2 + (my - cell[1]) ** 2
                if nearest_sq is None or distance_sq < nearest_sq:
                    nearest_sq = distance_sq
        if nearest_sq is None:
            return None
        return math.sqrt(nearest_sq) * resolution

    def _fallback_footprint_metrics(self) -> FootprintMetrics:
        half_length = 0.5 * self.robot_length_m
        half_width = 0.5 * self.robot_width_m
        return FootprintMetrics(
            inscribed_radius=(
                min(half_length, half_width) + self.footprint_padding_m
            ),
            circumscribed_radius=(
                math.hypot(half_length, half_width) +
                self.footprint_padding_m
            ),
            maximum_span=math.hypot(self.robot_length_m, self.robot_width_m),
            point_count=4,
        )

    @staticmethod
    def _footprint_metrics(
        msg: Optional[PolygonStamped],
    ) -> Optional[FootprintMetrics]:
        if msg is None or len(msg.polygon.points) < 3:
            return None
        points = [
            (float(point.x), float(point.y))
            for point in msg.polygon.points
        ]
        center_x = sum(point[0] for point in points) / len(points)
        center_y = sum(point[1] for point in points) / len(points)
        radii = [math.hypot(x - center_x, y - center_y) for x, y in points]
        edge_distances = []
        for first, second in zip(points, points[1:] + points[:1]):
            edge_dx = second[0] - first[0]
            edge_dy = second[1] - first[1]
            edge_length = math.hypot(edge_dx, edge_dy)
            if edge_length <= 1e-9:
                continue
            cross = abs(
                edge_dx * (center_y - first[1]) -
                edge_dy * (center_x - first[0])
            )
            edge_distances.append(cross / edge_length)
        spans = [
            math.hypot(ax - bx, ay - by)
            for index, (ax, ay) in enumerate(points)
            for bx, by in points[index + 1:]
        ]
        if not edge_distances or not spans:
            return None
        return FootprintMetrics(
            inscribed_radius=min(edge_distances),
            circumscribed_radius=max(radii),
            maximum_span=max(spans),
            point_count=len(points),
        )

    def _active_footprint_metrics(self) -> FootprintMetrics:
        return (
            self._footprint_metrics(self.global_footprint) or
            self._footprint_metrics(self.local_footprint) or
            self._fallback_footprint_metrics()
        )

    def _cost_description(self, value: int) -> str:
        if value < 0:
            return 'unknown'
        if value == 0:
            return 'free'
        if value >= self.lethal_cost_threshold:
            return 'lethal/inscribed'
        if value >= self.costmap_blocked_threshold:
            return 'inflated-blocked'
        return 'inflated-allowed'

    def _analyze_frontier_grid(
        self,
        label: str,
        grid: Optional[OccupancyGrid],
        goal_point: Point2,
        goal_frame: str,
        is_raw_map: bool = False,
    ) -> Tuple[bool, List[str]]:
        reasons: List[str] = []
        if grid is None:
            self.frontier_details[label] = 'not received'
            return False, [f'{label} unavailable']
        transformed = self._grid_point(goal_point, goal_frame, grid)
        if transformed is None:
            return False, [f'{label} transform unavailable']
        cell = self._world_to_cell(grid, transformed)
        if cell is None:
            self.frontier_details[label] = (
                'OUTSIDE frame=%s point=(%.3f,%.3f)' % (
                    grid.header.frame_id,
                    transformed[0],
                    transformed[1],
                )
            )
            return False, [f'frontier outside {label}']

        value = self._cell_value(grid, cell)
        if is_raw_map:
            nearest = self._nearest_occupied(
                grid,
                cell,
                self.map_occupied_threshold,
                self.wall_search_radius_m,
            )
            wall_distance = (
                'none-in-search-radius'
                if nearest is None else f'{nearest:.3f}m'
            )
            state = 'unknown' if value < 0 else (
                'wall' if value >= self.map_occupied_threshold else 'free'
            )
            on_wall = value >= self.map_occupied_threshold
            wall_adjacent = (
                nearest is not None and
                nearest <= self.frontier_wall_proximity_m
            )
            if value < 0:
                reasons.append('frontier is on unknown map cell')
            if on_wall:
                reasons.append('frontier is on wall cell')
            elif wall_adjacent:
                reasons.append('frontier is wall-adjacent')
            metrics = self._active_footprint_metrics()
            clearance_ok = (
                nearest is None or nearest >= metrics.inscribed_radius
            )
            if not clearance_ok:
                reasons.append(
                    'wall clearance is smaller than robot inscribed radius'
                )
            self.frontier_details[label] = (
                'cell=(%d,%d) occupancy=%d state=%s nearest_wall=%s '
                'wall_adjacent=%s required_inscribed_clearance=%.3fm '
                'clearance_ok=%s' % (
                    cell[0],
                    cell[1],
                    value,
                    state,
                    wall_distance,
                    wall_adjacent,
                    metrics.inscribed_radius,
                    clearance_ok,
                )
            )
            return not reasons, reasons

        state = self._cost_description(value)
        inside_inflation = 0 < value < self.lethal_cost_threshold
        blocked = value >= self.costmap_blocked_threshold
        if value < 0 and not self.allow_unknown_path:
            reasons.append(f'frontier is unknown in {label}')
        if blocked:
            reasons.append(f'frontier is blocked in {label} (cost={value})')
        self.frontier_details[label] = (
            'frame=%s cell=(%d,%d) cost=%d state=%s '
            'inside_inflation=%s blocked_at_%d=%s' % (
                grid.header.frame_id,
                cell[0],
                cell[1],
                value,
                state,
                inside_inflation,
                self.costmap_blocked_threshold,
                blocked,
            )
        )
        return not reasons, reasons

    def _cell_traversable(self, value: int) -> bool:
        if value < 0:
            return self.allow_unknown_path
        return value < self.path_blocked_threshold

    @staticmethod
    def _neighbors(cell: Cell) -> Iterable[Tuple[Cell, float]]:
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                yield (cell[0] + dx, cell[1] + dy), (
                    math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
                )

    def _diagonal_allowed(
        self,
        grid: OccupancyGrid,
        current: Cell,
        neighbor: Cell,
    ) -> bool:
        dx = neighbor[0] - current[0]
        dy = neighbor[1] - current[1]
        if dx == 0 or dy == 0:
            return True
        side_a = (current[0] + dx, current[1])
        side_b = (current[0], current[1] + dy)
        return (
            self._cell_traversable(self._cell_value(grid, side_a)) and
            self._cell_traversable(self._cell_value(grid, side_b))
        )

    def _grid_path(
        self,
        grid: OccupancyGrid,
        start: Cell,
        goal: Cell,
    ) -> Tuple[bool, float, int, str]:
        start_value = self._cell_value(grid, start)
        goal_value = self._cell_value(grid, goal)
        if not self._cell_traversable(start_value):
            return False, 0.0, 0, f'start blocked cost={start_value}'
        if not self._cell_traversable(goal_value):
            return False, 0.0, 0, f'goal blocked cost={goal_value}'

        width = int(grid.info.width)
        height = int(grid.info.height)
        resolution = float(grid.info.resolution)
        queue: List[Tuple[float, float, Cell]] = []
        heapq.heappush(queue, (0.0, 0.0, start))
        costs: Dict[Cell, float] = {start: 0.0}
        expanded = 0
        while queue and expanded < self.max_grid_path_expansions:
            _, current_cost, current = heapq.heappop(queue)
            if current_cost != costs.get(current):
                continue
            expanded += 1
            if current == goal:
                return True, current_cost * resolution, expanded, 'connected'
            for neighbor, step_cost in self._neighbors(current):
                if (
                    neighbor[0] < 0 or neighbor[1] < 0 or
                    neighbor[0] >= width or neighbor[1] >= height
                ):
                    continue
                neighbor_value = self._cell_value(grid, neighbor)
                if not self._cell_traversable(neighbor_value):
                    continue
                if not self._diagonal_allowed(grid, current, neighbor):
                    continue
                new_cost = current_cost + step_cost
                if new_cost >= costs.get(neighbor, math.inf):
                    continue
                costs[neighbor] = new_cost
                heuristic = math.hypot(
                    goal[0] - neighbor[0],
                    goal[1] - neighbor[1],
                )
                heapq.heappush(
                    queue,
                    (new_cost + heuristic, new_cost, neighbor),
                )
        reason = 'expansion limit reached' if queue else 'disconnected'
        return False, 0.0, expanded, reason

    def _probe_robot_passage(
        self,
        label: str,
        grid: Optional[OccupancyGrid],
    ) -> Optional[bool]:
        if grid is None:
            self.passage_details[label] = 'costmap not received'
            return None
        frame = grid.header.frame_id or self.map_frame
        robot = self._robot_pose(frame)
        if robot is None:
            self.passage_details[label] = 'robot TF unavailable'
            return None
        start = self._world_to_cell(grid, (robot[0], robot[1]))
        if start is None:
            self.passage_details[label] = 'robot outside costmap'
            return False
        center_cost = self._cell_value(grid, start)
        if not self._cell_traversable(center_cost):
            self.passage_details[label] = (
                'center blocked cell=%s cost=%d' % (start, center_cost)
            )
            return False

        resolution = float(grid.info.resolution)
        probe_cells = max(
            1,
            int(math.ceil(self.passage_probe_distance_m / resolution)),
        )
        width = int(grid.info.width)
        height = int(grid.info.height)
        queue = deque([start])
        visited = {start}
        max_distance_cells = 0.0
        free_neighbors = 0
        for neighbor, _ in self._neighbors(start):
            if (
                0 <= neighbor[0] < width and
                0 <= neighbor[1] < height and
                self._cell_traversable(self._cell_value(grid, neighbor))
            ):
                free_neighbors += 1
        while queue:
            current = queue.popleft()
            distance = math.hypot(
                current[0] - start[0],
                current[1] - start[1],
            )
            max_distance_cells = max(max_distance_cells, distance)
            if distance >= probe_cells:
                continue
            for neighbor, _ in self._neighbors(current):
                if neighbor in visited:
                    continue
                if (
                    neighbor[0] < 0 or neighbor[1] < 0 or
                    neighbor[0] >= width or neighbor[1] >= height
                ):
                    continue
                neighbor_value = self._cell_value(grid, neighbor)
                if not self._cell_traversable(neighbor_value):
                    continue
                if not self._diagonal_allowed(grid, current, neighbor):
                    continue
                visited.add(neighbor)
                queue.append(neighbor)
        reached = max_distance_cells >= probe_cells
        self.passage_details[label] = (
            'frame=%s pose=(%.3f,%.3f,%.1fdeg) cell=%s '
            'center_cost=%d free_neighbors=%d/8 connected_cells=%d '
            'max_reach=%.3fm probe=%.3fm passage=%s' % (
                frame,
                robot[0],
                robot[1],
                math.degrees(robot[2]),
                start,
                center_cost,
                free_neighbors,
                len(visited),
                max_distance_cells * resolution,
                self.passage_probe_distance_m,
                'PASS' if reached else 'BLOCKED',
            )
        )
        return reached

    def _check_grid_connectivity(
        self,
        goal_point: Point2,
        goal_frame: str,
    ) -> Tuple[bool, str]:
        grid = self.global_costmap
        if grid is None:
            return False, 'global costmap unavailable'
        frame = grid.header.frame_id or self.map_frame
        robot = self._robot_pose(frame)
        transformed_goal = self._transform_point(goal_point, goal_frame, frame)
        if robot is None or transformed_goal is None:
            return False, 'TF unavailable'
        start_cell = self._world_to_cell(grid, (robot[0], robot[1]))
        goal_cell = self._world_to_cell(grid, transformed_goal)
        if start_cell is None:
            return False, 'robot outside global costmap'
        if goal_cell is None:
            return False, 'frontier outside global costmap'
        found, length, expanded, reason = self._grid_path(
            grid,
            start_cell,
            goal_cell,
        )
        self.frontier_details['GRID_PATH'] = (
            'start=%s goal=%s found=%s length=%.3fm expanded=%d '
            'reason=%s unknown_allowed=%s threshold=%d' % (
                start_cell,
                goal_cell,
                found,
                length,
                expanded,
                reason,
                self.allow_unknown_path,
                self.path_blocked_threshold,
            )
        )
        return found, reason

    def _request_nav2_path(self, goal: GoalRecord):
        if not self.compute_path_enabled:
            return
        if self.path_request_generation == goal.generation:
            return
        if self.path_request_pending:
            return
        if not self.compute_path_client.wait_for_server(timeout_sec=0.0):
            self._set_condition(
                'NAV2_PATH_SERVER_UNAVAILABLE',
                True,
                'WARN',
                f'action unavailable: {self.compute_path_action}',
            )
            return
        self._clear_condition(
            'NAV2_PATH_SERVER_UNAVAILABLE',
            'ComputePathToPose server available',
        )

        request = ComputePathToPose.Goal()
        request.goal = goal.pose
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.planner_id = self.planner_id
        request.use_start = False
        generation = goal.generation
        self.path_request_pending = True
        self.path_request_generation = generation
        future = self.compute_path_client.send_goal_async(request)
        future.add_done_callback(
            lambda done: self._path_goal_response(done, generation)
        )

    def _path_goal_response(self, future, generation: int):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.path_request_pending = False
            self.nav2_path_summary = f'request failed: {exc}'
            self._transient_anomaly(
                'NAV2_PATH_REQUEST_FAILED',
                'ERROR',
                self.nav2_path_summary,
            )
            return
        if not goal_handle.accepted:
            self.path_request_pending = False
            self.nav2_path_summary = 'ComputePathToPose rejected'
            self._transient_anomaly(
                'NAV2_PATH_REJECTED',
                'ERROR',
                self.nav2_path_summary,
            )
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._path_result(done, generation)
        )

    def _path_result(self, future, generation: int):
        self.path_request_pending = False
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:  # noqa: BLE001
            self.nav2_path_summary = f'result failed: {exc}'
            self._transient_anomaly(
                'NAV2_PATH_RESULT_FAILED',
                'ERROR',
                self.nav2_path_summary,
            )
            return
        poses = list(result.path.poses) if result.path is not None else []
        length = self._path_length(poses)
        status_ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED
        error_code = int(getattr(result, 'error_code', 0))
        error_msg = str(getattr(result, 'error_msg', ''))
        valid = status_ok and error_code == 0 and bool(poses)
        stale = self.goal is None or generation != self.goal.generation
        self.nav2_path_summary = (
            'valid=%s status=%d error_code=%d error_msg=%r '
            'poses=%d length=%.3fm planning_time=%.4fs stale=%s' % (
                valid,
                wrapped.status,
                error_code,
                error_msg,
                len(poses),
                length,
                float(result.planning_time.sec) +
                float(result.planning_time.nanosec) * 1e-9,
                stale,
            )
        )
        self._set_condition(
            'NAV2_PATH_INVALID',
            not valid and not stale,
            'ERROR',
            self.nav2_path_summary,
        )

    @staticmethod
    def _path_length(poses: Sequence) -> float:
        total = 0.0
        for first, second in zip(poses, poses[1:]):
            total += math.hypot(
                second.pose.position.x - first.pose.position.x,
                second.pose.position.y - first.pose.position.y,
            )
        return total

    def _dwb_callback(self, msg: LocalPlanEvaluation):
        self.dwb_message_received = True
        if not msg.twists:
            self.dwb_summary = 'evaluation contains no trajectories'
            self.dwb_valid_count = 0
            self._set_condition(
                'DWB_NO_VALID_TRAJECTORY',
                self.goal_active,
                'ERROR',
                self.dwb_summary,
            )
            return
        best_index = int(msg.best_index)
        if best_index < 0 or best_index >= len(msg.twists):
            self.dwb_summary = (
                'invalid best_index=%d trajectories=%d' % (
                    best_index,
                    len(msg.twists),
                )
            )
            self._set_condition(
                'DWB_INVALID_SELECTION',
                True,
                'ERROR',
                self.dwb_summary,
            )
            return
        self._clear_condition('DWB_INVALID_SELECTION', 'best index valid')
        best = msg.twists[best_index]
        critics = sorted(
            (
                (score.name, float(score.raw_score), float(score.scale),
                 float(score.raw_score) * float(score.scale))
                for score in best.scores
            ),
            key=lambda item: abs(item[3]),
            reverse=True,
        )
        critic_text = ', '.join(
            '%s(raw=%.3f scale=%.3f contribution=%.3f)' % item
            for item in critics
        )
        velocity = best.traj.velocity
        invalid_count = sum(1 for score in msg.twists if score.total < 0.0)
        self.dwb_valid_count = len(msg.twists) - invalid_count
        self.dwb_summary = (
            'selected index=%d/%d velocity=(vx=%.3f,vy=%.3f,wz=%.3f) '
            'total=%.3f invalid=%d worst_index=%d critics=[%s]' % (
                best_index,
                len(msg.twists),
                velocity.x,
                velocity.y,
                velocity.theta,
                best.total,
                invalid_count,
                int(msg.worst_index),
                critic_text,
            )
        )
        self._set_condition(
            'DWB_NO_VALID_TRAJECTORY',
            self.goal_active and self.dwb_valid_count <= 0,
            'ERROR',
            self.dwb_summary,
        )

    def _path_callback(self, label: str, msg: Path):
        now_ns = self.get_clock().now().nanoseconds
        self.path_summaries[label] = (
            len(msg.poses),
            self._path_length(msg.poses),
            now_ns,
        )
        anomaly_key = 'LOCAL_PATH_EMPTY' if 'local_plan' in label else (
            'TRANSFORMED_GLOBAL_PATH_EMPTY'
        )
        self._set_condition(
            anomaly_key,
            self.goal_active and not msg.poses,
            'ERROR',
            f'{label} frame={msg.header.frame_id} poses={len(msg.poses)}',
        )

    def _cmd_callback(self, label: str, msg: Twist):
        self.last_cmd[label] = msg
        self.last_cmd_receive_ns[label] = self.get_clock().now().nanoseconds

    def _record_runtime_sample(self, now_ns: int):
        robot = self._robot_pose(self.map_frame)
        goal_distance = self.last_feedback_distance
        if (
            goal_distance is None and
            robot is not None and
            self.goal is not None
        ):
            goal_point = self._transform_point(
                (
                    float(self.goal.pose.pose.position.x),
                    float(self.goal.pose.pose.position.y),
                ),
                self.goal.pose.header.frame_id,
                self.map_frame,
            )
            if goal_point is not None:
                goal_distance = math.hypot(
                    goal_point[0] - robot[0],
                    goal_point[1] - robot[1],
                )
        command = (
            self.last_cmd.get('smoothed') or
            self.last_cmd.get('controller')
        )
        cmd_linear, cmd_angular = self._twist_values(command)
        odom_linear = 0.0
        odom_angular = 0.0
        if self.last_odom is not None:
            twist = self.last_odom.twist.twist
            odom_linear = math.hypot(twist.linear.x, twist.linear.y)
            odom_angular = abs(float(twist.angular.z))
        self.history.append(RuntimeSample(
            stamp_ns=now_ns,
            goal_generation=self.goal_generation,
            robot_pose=robot,
            goal_distance=goal_distance,
            cmd_linear=cmd_linear,
            cmd_angular=cmd_angular,
            odom_linear=odom_linear,
            odom_angular=odom_angular,
        ))
        cutoff_ns = now_ns - int(self.history_window_s * 1e9)
        while self.history and self.history[0].stamp_ns < cutoff_ns:
            self.history.popleft()

    def _window_samples(self, duration_s: float) -> List[RuntimeSample]:
        if not self.history:
            return []
        cutoff_ns = self.history[-1].stamp_ns - int(duration_s * 1e9)
        return [
            sample for sample in self.history
            if sample.stamp_ns >= cutoff_ns and
            sample.goal_generation == self.goal_generation
        ]

    @staticmethod
    def _window_motion(
        samples: Sequence[RuntimeSample],
    ) -> Tuple[Optional[float], Optional[float], Optional[float]]:
        if len(samples) < 2:
            return None, None, None
        first = samples[0]
        last = samples[-1]
        pose_delta = None
        yaw_delta = None
        if first.robot_pose is not None and last.robot_pose is not None:
            pose_delta = math.hypot(
                last.robot_pose[0] - first.robot_pose[0],
                last.robot_pose[1] - first.robot_pose[1],
            )
            yaw_delta = abs(math.atan2(
                math.sin(last.robot_pose[2] - first.robot_pose[2]),
                math.cos(last.robot_pose[2] - first.robot_pose[2]),
            ))
        goal_progress = None
        if first.goal_distance is not None and last.goal_distance is not None:
            goal_progress = first.goal_distance - last.goal_distance
        return pose_delta, yaw_delta, goal_progress

    def _window_ready(
        self,
        samples: Sequence[RuntimeSample],
        duration_s: float,
    ) -> bool:
        if len(samples) < 2:
            return False
        elapsed = (samples[-1].stamp_ns - samples[0].stamp_ns) * 1e-9
        return elapsed >= max(0.0, duration_s - self.diagnostic_period_s * 1.5)

    def _evaluate_motion_conditions(self):
        no_progress_samples = self._window_samples(self.no_progress_timeout_s)
        _, _, goal_progress = self._window_motion(no_progress_samples)
        far_from_goal = (
            self.last_feedback_distance is None or
            self.last_feedback_distance > 0.20
        )
        no_progress = (
            self.goal_active and
            far_from_goal and
            self._window_ready(
                no_progress_samples,
                self.no_progress_timeout_s,
            ) and
            goal_progress is not None and
            goal_progress < self.min_goal_progress_m
        )
        progress_detail = (
            'goal progress %.3fm over %.1fs; minimum %.3fm' % (
                goal_progress if goal_progress is not None else math.nan,
                self.no_progress_timeout_s,
                self.min_goal_progress_m,
            )
        )
        self._set_condition(
            'NO_PROGRESS',
            no_progress,
            'ERROR',
            progress_detail,
        )

        stuck_samples = self._window_samples(self.stuck_timeout_s)
        pose_delta, yaw_delta, _ = self._window_motion(stuck_samples)
        command_active = any(
            sample.cmd_linear >= self.linear_cmd_threshold or
            sample.cmd_angular >= self.angular_cmd_threshold
            for sample in stuck_samples
        )
        stuck = (
            self.goal_active and
            self._window_ready(stuck_samples, self.stuck_timeout_s) and
            command_active and
            pose_delta is not None and
            yaw_delta is not None and
            pose_delta < self.min_pose_progress_m and
            yaw_delta < self.min_yaw_progress_rad
        )
        stuck_detail = (
            'command active, pose_delta=%s yaw_delta=%s over %.1fs' % (
                pose_delta,
                yaw_delta,
                self.stuck_timeout_s,
            )
        )
        self._set_condition('ROBOT_STUCK', stuck, 'ERROR', stuck_detail)

        zero_samples = self._window_samples(self.zero_cmd_timeout_s)
        zero_command = bool(zero_samples) and all(
            sample.cmd_linear < self.linear_cmd_threshold and
            sample.cmd_angular < self.angular_cmd_threshold
            for sample in zero_samples
        )
        controller_zero = (
            self.goal_active and
            far_from_goal and
            self._window_ready(zero_samples, self.zero_cmd_timeout_s) and
            zero_command
        )
        self._set_condition(
            'CONTROLLER_ZERO_COMMAND',
            controller_zero,
            'WARN',
            'goal active but controller command stayed near zero for %.1fs' %
            self.zero_cmd_timeout_s,
        )

    def _evaluate_data_staleness(self, now_ns: int):
        grace_elapsed = (
            now_ns - self.start_time_ns >=
            int(self.startup_grace_period_s * 1e9)
        )
        if not grace_elapsed or not self.goal_active:
            detail = 'goal inactive or startup grace'
            self._clear_condition('DATA_STALE', detail)
            self._clear_condition('TF_UNAVAILABLE', detail)
            return
        limit_ns = int(self.data_stale_timeout_s * 1e9)
        stamps = {
            'map': self.grid_receive_ns.get('map', 0),
            'global_costmap': self.grid_receive_ns.get('global', 0),
            'local_costmap': self.grid_receive_ns.get('local', 0),
            'odom': self.last_odom_ns,
            'scan': self.last_scan_ns,
        }
        stale = [
            name for name, stamp in stamps.items()
            if stamp <= 0 or now_ns - stamp > limit_ns
        ]
        self._set_condition(
            'DATA_STALE',
            bool(stale),
            'WARN',
            'stale topics=' + ','.join(stale),
        )
        self._set_condition(
            'TF_UNAVAILABLE',
            self.tf_issue is not None,
            'ERROR',
            self.tf_issue or 'TF restored',
        )

    def _diagnostic_tick(self):
        now_ns = self.get_clock().now().nanoseconds
        self._record_runtime_sample(now_ns)
        self._evaluate_data_staleness(now_ns)
        self._evaluate_motion_conditions()

        global_passage = self._probe_robot_passage(
            'global',
            self.global_costmap,
        )
        local_passage = self._probe_robot_passage(
            'local',
            self.local_costmap,
        )
        self._set_condition(
            'ROBOT_GLOBAL_BLOCKED',
            self.goal_active and global_passage is False,
            'ERROR',
            self.passage_details.get('global', 'unavailable'),
        )
        self._set_condition(
            'ROBOT_LOCAL_BLOCKED',
            self.goal_active and local_passage is False,
            'ERROR',
            self.passage_details.get('local', 'unavailable'),
        )

        if self.goal is None:
            return
        if self.map_msg is None or self.global_costmap is None:
            return

        goal = self.goal
        goal_point = (
            float(goal.pose.pose.position.x),
            float(goal.pose.pose.position.y),
        )
        reasons: List[str] = []
        _, map_reasons = self._analyze_frontier_grid(
            'MAP',
            self.map_msg,
            goal_point,
            goal.pose.header.frame_id,
            is_raw_map=True,
        )
        reasons.extend(map_reasons)
        _, global_reasons = self._analyze_frontier_grid(
            'GLOBAL_COSTMAP',
            self.global_costmap,
            goal_point,
            goal.pose.header.frame_id,
        )
        reasons.extend(global_reasons)
        if self.local_costmap is not None:
            _, local_reasons = self._analyze_frontier_grid(
                'LOCAL_COSTMAP',
                self.local_costmap,
                goal_point,
                goal.pose.header.frame_id,
            )
            reasons.extend(
                reason for reason in local_reasons
                if 'outside LOCAL_COSTMAP' not in reason
            )

        if self.last_analyzed_generation != goal.generation:
            connected, path_reason = self._check_grid_connectivity(
                goal_point,
                goal.pose.header.frame_id,
            )
            self.grid_connectivity_failure = None if connected else path_reason
            self.last_analyzed_generation = goal.generation
        if self.grid_connectivity_failure is not None:
            reasons.append(
                'no global costmap connection: ' +
                self.grid_connectivity_failure
            )

        reason_text = (
            'none' if not reasons else '; '.join(sorted(set(reasons)))
        )
        self._set_condition(
            'FRONTIER_INVALID',
            bool(reasons),
            'ERROR',
            reason_text,
        )
        self._request_nav2_path(goal)

        dwb_grace_elapsed = (
            self.goal_active and
            now_ns - self.goal_set_ns > int(self.startup_grace_period_s * 1e9)
        )
        self._set_condition(
            'DWB_EVALUATION_MISSING',
            dwb_grace_elapsed and not self.dwb_message_received,
            'WARN',
            'no DWB /evaluation received; verify planner type and topic',
        )

        if not self.anomaly_only_logging:
            self.get_logger().info(
                '[HEARTBEAT] goal=%s active=%s anomalies=%s' % (
                    goal_point,
                    self.goal_active,
                    sorted(self.active_anomalies),
                )
            )


def main(args=None):
    """Run the diagnostics node until interrupted."""
    rclpy.init(args=args)
    node = FrontierDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
