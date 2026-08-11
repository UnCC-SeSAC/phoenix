from __future__ import annotations

import json
import math
from dataclasses import dataclass

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .nav2_navigator import Nav2Navigator


@dataclass(frozen=True)
class PendingGoal:
    action_id: str
    target_id: str | None


class VLANavigationBridgeNode(Node):
    """Humble-side bridge between neutral VLA topics and Nav2 actions.

    The Jazzy VLA Brain publishes JSON goals. This node translates them to
    Nav2 ``NavigateToPose`` and publishes normalized terminal results.
    """

    def __init__(self) -> None:
        super().__init__("vla_navigation_bridge")
        self.declare_parameter("goal_topic", "/vla/navigation_goal")
        self.declare_parameter("result_topic", "/vla/navigation_result")
        self.declare_parameter("cancel_topic", "/vla/navigation_cancel")
        self.declare_parameter("robot_pose_topic", "/vla/robot_pose_json")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("pose_publish_period_sec", 0.2)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.navigator = Nav2Navigator(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.result_pub = self.create_publisher(
            String,
            str(self.get_parameter("result_topic").value),
            10,
        )
        self.pose_pub = self.create_publisher(
            String,
            str(self.get_parameter("robot_pose_topic").value),
            10,
        )
        self.goal_sub = self.create_subscription(
            String,
            str(self.get_parameter("goal_topic").value),
            self._goal_callback,
            10,
        )
        self.cancel_sub = self.create_subscription(
            String,
            str(self.get_parameter("cancel_topic").value),
            self._cancel_callback,
            10,
        )
        self.pose_timer = self.create_timer(
            max(0.05, float(self.get_parameter("pose_publish_period_sec").value)),
            self._publish_robot_pose,
        )

        self.pending: PendingGoal | None = None
        self.completed_results: dict[str, dict] = {}
        self.get_logger().info(
            "VLA navigation bridge started: JSON topic -> NavigateToPose"
        )

    def _goal_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            action_id = str(data["action_id"])
            target_id_raw = data.get("target_id")
            target_id = str(target_id_raw) if target_id_raw is not None else None
            pose_data = data["target_pose"]
            frame_id = str(data.get("frame_id", self.map_frame))
            x = float(pose_data["x"])
            y = float(pose_data["y"])
            yaw = float(pose_data.get("yaw", 0.0))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid VLA navigation goal: {exc}")
            return

        if action_id in self.completed_results:
            self._publish_result(self.completed_results[action_id])
            return
        if self.pending is not None:
            self._publish_terminal(
                action_id,
                target_id,
                "FAILED",
                "Nav2 goal이 이미 실행 중입니다.",
            )
            return
        if frame_id != self.map_frame:
            self._publish_terminal(
                action_id,
                target_id,
                "FAILED",
                f"지원하지 않는 frame_id={frame_id}; expected={self.map_frame}",
            )
            return

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map_frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.orientation.w = math.cos(yaw * 0.5)

        self.pending = PendingGoal(action_id, target_id)
        self.navigator.navigate(goal, self._navigation_done)
        self.get_logger().info(
            f"NavigateToPose submitted: action_id={action_id}, target={target_id}, "
            f"pose=({x:.2f}, {y:.2f}, {yaw:.2f})"
        )

    def _cancel_callback(self, msg: String) -> None:
        try:
            requested = str(json.loads(msg.data)["action_id"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        if self.pending is None or self.pending.action_id != requested:
            return
        if not self.navigator.cancel_navigation():
            self.get_logger().warning("Nav2 cancel request could not be submitted")

    def _navigation_done(self, nav_status: int) -> None:
        pending = self.pending
        self.pending = None
        if pending is None:
            return

        status = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED: "ABORTED",
            GoalStatus.STATUS_CANCELED: "CANCELED",
        }.get(nav_status, "FAILED")
        self._publish_terminal(
            pending.action_id,
            pending.target_id,
            status,
            f"Nav2 GoalStatus={nav_status}",
        )

    def _publish_terminal(
        self,
        action_id: str,
        target_id: str | None,
        status: str,
        message: str,
    ) -> None:
        payload = {
            "action_id": action_id,
            "target_id": target_id,
            "status": status,
            "message": message,
        }
        if status in {"SUCCEEDED", "FAILED", "ABORTED", "CANCELED", "TIMED_OUT"}:
            self.completed_results[action_id] = payload
            if len(self.completed_results) > 256:
                oldest = next(iter(self.completed_results))
                self.completed_results.pop(oldest, None)
        self._publish_result(payload)

    def _publish_result(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.result_pub.publish(msg)

    def _publish_robot_pose(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return

        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        payload = {
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "frame_id": self.map_frame,
            "pose": {
                "x": float(tf.transform.translation.x),
                "y": float(tf.transform.translation.y),
                "yaw": float(yaw),
            },
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pose_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VLANavigationBridgeNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
