import math
import threading
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .failure_memory import FailureMemory
from .frontier_detector import FrontierDetector
from .map_utils import (
    GridMap,
    path_average_cost,
    path_length,
    path_max_cost,
)
from .models import Frontier, Pose2D
from .nav2_navigator import Nav2Navigator
from .priority_calculator import PriorityCalculator


class ExplorerNode(Node):
    STATE_WAITING = 'WAITING'
    STATE_EVALUATING = 'EVALUATING'
    STATE_NAVIGATING = 'NAVIGATING'
    STATE_CANCELING = 'CANCELING'

    def __init__(self):
        super().__init__('explorer_node')

        # Topics / frames
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter(
            'global_costmap_topic',
            '/global_costmap/costmap',
        )
        self.declare_parameter(
            'local_costmap_topic',
            '/local_costmap/costmap',
        )
        self.declare_parameter('hazard_map_topic', '/hazard_map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # Frontier
        self.declare_parameter('free_threshold', 20)
        self.declare_parameter('min_frontier_cluster_size', 8)
        self.declare_parameter('max_frontiers', 100)
        self.declare_parameter('max_candidates_to_plan', 8)
        self.declare_parameter('min_goal_distance', 0.45)
        self.declare_parameter('max_goal_distance', 10.0)

        # Timing / Raspberry Pi friendly defaults
        self.declare_parameter('exploration_period', 3.0)
        self.declare_parameter('replan_delay_after_goal', 1.5)

        # Safety
        self.declare_parameter('safety_check_period', 0.5)
        self.declare_parameter('local_safety_radius', 0.18)
        self.declare_parameter('local_stop_threshold', 0.98)
        self.declare_parameter('hazard_stop_threshold', 0.85)

        # Frontier evaluation
        self.declare_parameter('goal_narrowness_radius', 0.40)
        self.declare_parameter('unknown_global_cost', 0.70)
        self.declare_parameter('minimum_score', -10.0)

        # Priority weights
        self.declare_parameter('information_gain_weight', 2.0)
        self.declare_parameter('mission_value_weight', 1.5)
        self.declare_parameter('path_cost_weight', 0.8)
        self.declare_parameter('hazard_weight', 2.0)
        self.declare_parameter('global_cost_weight', 0.5)
        self.declare_parameter('narrowness_weight', 0.5)
        self.declare_parameter('failure_weight', 2.0)
        self.declare_parameter('info_reference_m', 2.0)
        self.declare_parameter('path_reference_m', 8.0)

        # Failure memory
        self.declare_parameter('failure_radius', 0.60)
        self.declare_parameter('failure_ttl', 120.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.global_costmap_topic = self.get_parameter(
            'global_costmap_topic'
        ).value
        self.local_costmap_topic = self.get_parameter(
            'local_costmap_topic'
        ).value
        self.hazard_map_topic = self.get_parameter(
            'hazard_map_topic'
        ).value

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.max_frontiers = int(
            self.get_parameter('max_frontiers').value
        )
        self.max_candidates_to_plan = int(
            self.get_parameter('max_candidates_to_plan').value
        )
        self.min_goal_distance = float(
            self.get_parameter('min_goal_distance').value
        )
        self.max_goal_distance = float(
            self.get_parameter('max_goal_distance').value
        )
        self.replan_delay_after_goal = float(
            self.get_parameter('replan_delay_after_goal').value
        )
        self.goal_narrowness_radius = float(
            self.get_parameter('goal_narrowness_radius').value
        )
        self.unknown_global_cost = float(
            self.get_parameter('unknown_global_cost').value
        )
        self.minimum_score = float(
            self.get_parameter('minimum_score').value
        )

        self.local_safety_radius = float(
            self.get_parameter('local_safety_radius').value
        )
        self.local_stop_threshold = float(
            self.get_parameter('local_stop_threshold').value
        )
        self.hazard_stop_threshold = float(
            self.get_parameter('hazard_stop_threshold').value
        )

        self.frontier_detector = FrontierDetector(
            free_threshold=int(
                self.get_parameter('free_threshold').value
            ),
            min_cluster_size=int(
                self.get_parameter(
                    'min_frontier_cluster_size'
                ).value
            ),
        )

        self.priority = PriorityCalculator(
            information_gain_weight=self.get_parameter(
                'information_gain_weight'
            ).value,
            mission_value_weight=self.get_parameter(
                'mission_value_weight'
            ).value,
            path_cost_weight=self.get_parameter(
                'path_cost_weight'
            ).value,
            hazard_weight=self.get_parameter(
                'hazard_weight'
            ).value,
            global_cost_weight=self.get_parameter(
                'global_cost_weight'
            ).value,
            narrowness_weight=self.get_parameter(
                'narrowness_weight'
            ).value,
            failure_weight=self.get_parameter(
                'failure_weight'
            ).value,
            info_reference_m=self.get_parameter(
                'info_reference_m'
            ).value,
            path_reference_m=self.get_parameter(
                'path_reference_m'
            ).value,
        )

        self.failure_memory = FailureMemory(
            radius=self.get_parameter('failure_radius').value,
            ttl=self.get_parameter('failure_ttl').value,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.navigator = Nav2Navigator(self)

        self.map_grid: Optional[GridMap] = None
        self.global_costmap: Optional[GridMap] = None
        self.local_costmap: Optional[GridMap] = None
        self.hazard_map: Optional[GridMap] = None

        self.state = self.STATE_WAITING
        self.state_lock = threading.RLock()
        self.next_cycle_time_ns = 0

        self.eval_queue: List[Frontier] = []
        self.evaluated: List[Frontier] = []
        self.current_eval_frontier: Optional[Frontier] = None
        self.current_goal_frontier: Optional[Frontier] = None

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            transient_qos,
        )

        self.global_cost_sub = self.create_subscription(
            OccupancyGrid,
            self.global_costmap_topic,
            self._global_costmap_callback,
            transient_qos,
        )

        self.local_cost_sub = self.create_subscription(
            OccupancyGrid,
            self.local_costmap_topic,
            self._local_costmap_callback,
            transient_qos,
        )

        self.hazard_sub = self.create_subscription(
            OccupancyGrid,
            self.hazard_map_topic,
            self._hazard_callback,
            transient_qos,
        )

        self.frontier_pub = self.create_publisher(
            MarkerArray,
            '/exploration/frontiers',
            10,
        )

        self.best_pub = self.create_publisher(
            Marker,
            '/exploration/best_frontier',
            10,
        )

        self.state_pub = self.create_publisher(
            String,
            '/exploration/state',
            10,
        )

        exploration_period = max(
            1.0,
            float(self.get_parameter('exploration_period').value),
        )
        safety_period = max(
            0.1,
            float(self.get_parameter('safety_check_period').value),
        )

        self.exploration_timer = self.create_timer(
            exploration_period,
            self._exploration_tick,
        )

        self.safety_timer = self.create_timer(
            safety_period,
            self._safety_tick,
        )

        self.get_logger().info(
            'ExplorerNode started. Frontier=/map, '
            'reachability=Nav2, safety=local_costmap+hazard_map.'
        )

    # ------------------------------------------------------------------
    # Subscribers
    # ------------------------------------------------------------------

    def _map_callback(self, msg):
        self.map_grid = GridMap.from_msg(msg)

    def _global_costmap_callback(self, msg):
        self.global_costmap = GridMap.from_msg(msg)

    def _local_costmap_callback(self, msg):
        self.local_costmap = GridMap.from_msg(msg)

    def _hazard_callback(self, msg):
        self.hazard_map = GridMap.from_msg(msg)

    # ------------------------------------------------------------------
    # TF / poses
    # ------------------------------------------------------------------

    def _robot_pose(self, target_frame: str) -> Optional[Pose2D]:
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.15),
            )
        except TransformException:
            return None

        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        return Pose2D(
            x=float(tf.transform.translation.x),
            y=float(tf.transform.translation.y),
            yaw=float(yaw),
        )

    def _pose_stamped(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        msg.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.orientation.w = math.cos(yaw * 0.5)
        return msg

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------

    def _set_state(self, state: str):
        with self.state_lock:
            self.state = state

        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def _can_start_cycle(self) -> bool:
        if self.state != self.STATE_WAITING:
            return False

        if self.get_clock().now().nanoseconds < self.next_cycle_time_ns:
            return False

        if self.map_grid is None:
            return False

        if self.global_costmap is None:
            return False

        if not self.navigator.planner_ready():
            self.get_logger().info(
                'Waiting for Nav2 /compute_path_to_pose...',
                throttle_duration_sec=5.0,
            )
            return False

        if not self.navigator.navigator_ready():
            self.get_logger().info(
                'Waiting for Nav2 /navigate_to_pose...',
                throttle_duration_sec=5.0,
            )
            return False

        return True

    # ------------------------------------------------------------------
    # Exploration cycle
    # ------------------------------------------------------------------

    def _exploration_tick(self):
        with self.state_lock:
            if not self._can_start_cycle():
                return
            self._set_state(self.STATE_EVALUATING)

        robot = self._robot_pose(self.map_frame)
        if robot is None:
            self.get_logger().warn(
                f'No TF {self.map_frame} -> {self.base_frame}',
                throttle_duration_sec=5.0,
            )
            self._set_state(self.STATE_WAITING)
            return

        frontiers = self.frontier_detector.find_frontiers(
            self.map_grid,
            max_frontiers=self.max_frontiers,
        )

        filtered = []
        for frontier in frontiers:
            d = math.hypot(
                frontier.x - robot.x,
                frontier.y - robot.y,
            )
            frontier.euclidean_distance = d

            if d < self.min_goal_distance:
                continue

            if self.max_goal_distance > 0 and d > self.max_goal_distance:
                continue

            frontier.failure_penalty = self.failure_memory.penalty(
                frontier.x,
                frontier.y,
            )
            filtered.append(frontier)

        self._publish_frontiers(filtered)

        if not filtered:
            self.get_logger().info(
                'No valid frontiers. Exploration may be complete.'
            )
            self._set_state(self.STATE_WAITING)
            self._schedule_next_cycle(3.0)
            return

        # Cheap pre-sort before expensive Nav2 planning.
        filtered.sort(
            key=lambda f: (
                -f.information_gain,
                f.euclidean_distance,
            )
        )

        self.eval_queue = filtered[
            : max(1, self.max_candidates_to_plan)
        ]
        self.evaluated = []

        self.get_logger().info(
            f'Frontiers={len(filtered)}, '
            f'planning top {len(self.eval_queue)} candidates.'
        )

        self._request_next_plan(robot)

    def _request_next_plan(self, robot: Optional[Pose2D] = None):
        if self.state != self.STATE_EVALUATING:
            return

        if not self.eval_queue:
            self._finish_evaluation()
            return

        if robot is None:
            robot = self._robot_pose(self.map_frame)

        if robot is None:
            self._set_state(self.STATE_WAITING)
            self._schedule_next_cycle(1.0)
            return

        frontier = self.eval_queue.pop(0)
        self.current_eval_frontier = frontier

        yaw = math.atan2(
            frontier.y - robot.y,
            frontier.x - robot.x,
        )

        start = self._pose_stamped(
            robot.x,
            robot.y,
            robot.yaw,
        )
        goal = self._pose_stamped(
            frontier.x,
            frontier.y,
            yaw,
        )

        self.navigator.request_path(
            start,
            goal,
            lambda path, f=frontier: self._on_path_result(f, path),
        )

    def _on_path_result(self, frontier: Frontier, path):
        if self.state != self.STATE_EVALUATING:
            return

        if path is not None and len(path.poses) > 0:
            frontier.path_length = path_length(path)

            frontier.global_cost = path_average_cost(
                path,
                self.global_costmap,
                unknown_cost=self.unknown_global_cost,
            )

            frontier.hazard_risk = path_max_cost(
                path,
                self.hazard_map,
                unknown_cost=0.0,
            )

            if self.global_costmap is not None:
                frontier.narrowness = (
                    self.global_costmap.occupied_ratio_in_radius(
                        frontier.x,
                        frontier.y,
                        self.goal_narrowness_radius,
                    )
                )

            self.priority.calculate_score(frontier)
            self.evaluated.append(frontier)

            self.get_logger().info(
                f'Frontier {frontier.frontier_id}: '
                f'score={frontier.score:.3f}, '
                f'path={frontier.path_length:.2f}m, '
                f'hazard={frontier.hazard_risk:.2f}, '
                f'global={frontier.global_cost:.2f}'
            )
        else:
            self.get_logger().info(
                f'Frontier {frontier.frontier_id}: unreachable.'
            )

        self.current_eval_frontier = None
        self._request_next_plan()

    def _finish_evaluation(self):
        reachable = [
            f
            for f in self.evaluated
            if f.reachable() and f.score >= self.minimum_score
        ]

        if not reachable:
            self.get_logger().warn(
                'No reachable frontier candidate this cycle.'
            )
            self._set_state(self.STATE_WAITING)
            self._schedule_next_cycle(2.0)
            return

        best = max(reachable, key=lambda f: f.score)
        self.current_goal_frontier = best
        self._publish_best_frontier(best)

        robot = self._robot_pose(self.map_frame)
        if robot is None:
            self._set_state(self.STATE_WAITING)
            self._schedule_next_cycle(1.0)
            return

        yaw = math.atan2(
            best.y - robot.y,
            best.x - robot.x,
        )

        goal = self._pose_stamped(
            best.x,
            best.y,
            yaw,
        )

        self.get_logger().info(
            f'Navigate -> Frontier {best.frontier_id}: '
            f'({best.x:.2f}, {best.y:.2f}), '
            f'score={best.score:.3f}'
        )

        self._set_state(self.STATE_NAVIGATING)

        self.navigator.navigate(
            goal,
            self._on_navigation_result,
        )

    def _on_navigation_result(self, status: int):
        frontier = self.current_goal_frontier
        self.current_goal_frontier = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'Frontier goal reached. Recomputing from latest /map.'
            )
        else:
            self.get_logger().warn(
                f'Frontier navigation ended with status={status}.'
            )
            if frontier is not None:
                self.failure_memory.record(
                    frontier.x,
                    frontier.y,
                )

        self._set_state(self.STATE_WAITING)
        self._schedule_next_cycle(self.replan_delay_after_goal)

    def _schedule_next_cycle(self, seconds: float):
        self.next_cycle_time_ns = (
            self.get_clock().now().nanoseconds
            + int(max(0.0, seconds) * 1e9)
        )

    # ------------------------------------------------------------------
    # Safety monitor
    # ------------------------------------------------------------------

    def _safety_tick(self):
        if self.state != self.STATE_NAVIGATING:
            return

        # Local costmap is in odom in the current Hiwonder configuration.
        if self.local_costmap is not None:
            local_robot = self._robot_pose(
                self.local_costmap.frame_id or 'odom'
            )
            if local_robot is not None:
                local_risk = self.local_costmap.max_cost_in_radius(
                    local_robot.x,
                    local_robot.y,
                    self.local_safety_radius,
                    unknown_cost=0.0,
                )
                if local_risk >= self.local_stop_threshold:
                    self._cancel_for_safety(
                        f'local_costmap risk={local_risk:.2f}'
                    )
                    return

        if self.hazard_map is not None:
            robot = self._robot_pose(
                self.hazard_map.frame_id or self.map_frame
            )
            if robot is not None:
                hazard = self.hazard_map.max_cost_in_radius(
                    robot.x,
                    robot.y,
                    self.local_safety_radius,
                    unknown_cost=0.0,
                )
                if hazard >= self.hazard_stop_threshold:
                    self._cancel_for_safety(
                        f'hazard risk={hazard:.2f}'
                    )

    def _cancel_for_safety(self, reason: str):
        if self.state != self.STATE_NAVIGATING:
            return

        self.get_logger().warn(
            f'Safety cancel: {reason}'
        )
        self._set_state(self.STATE_CANCELING)

        frontier = self.current_goal_frontier
        if frontier is not None:
            self.failure_memory.record(frontier.x, frontier.y)

        cancelled = self.navigator.cancel_navigation()

        if not cancelled:
            self._set_state(self.STATE_WAITING)
            self._schedule_next_cycle(1.0)

    # ------------------------------------------------------------------
    # RViz markers
    # ------------------------------------------------------------------

    def _publish_frontiers(self, frontiers: List[Frontier]):
        array = MarkerArray()

        delete = Marker()
        delete.action = Marker.DELETEALL
        array.markers.append(delete)

        stamp = self.get_clock().now().to_msg()

        for i, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = stamp
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = frontier.x
            marker.pose.position.y = frontier.y
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12

            # No fixed colors are required for algorithmic operation.
            # RViz still needs valid RGBA values.
            marker.color.r = 0.2
            marker.color.g = 0.6
            marker.color.b = 1.0
            marker.color.a = 0.9

            array.markers.append(marker)

        self.frontier_pub.publish(array)

    def _publish_best_frontier(self, frontier: Frontier):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'best_frontier'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = frontier.x
        marker.pose.position.y = frontier.y
        marker.pose.position.z = 0.08
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22

        marker.color.r = 0.2
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 1.0

        self.best_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
