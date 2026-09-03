#!/usr/bin/env python3
"""Bridge validated VLA spray topic commands to the SuppressFire action."""

from __future__ import annotations

import json

import rclpy
from action_msgs.msg import GoalStatus
from interfaces.action import SuppressFire
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class VLASprayBridge(Node):
    def __init__(self) -> None:
        super().__init__('vla_spray_bridge')
        self.declare_parameter('max_attempts_per_command', 1)
        self._max_attempts = int(
            self.get_parameter('max_attempts_per_command').value
        )
        self._client = ActionClient(self, SuppressFire, 'suppress_fire')
        self._result_pub = self.create_publisher(String, '/vla/spray_result', 10)
        self.create_subscription(String, '/vla/spray_command', self._on_command, 10)
        self.create_subscription(String, '/vla/spray_cancel', self._on_cancel, 10)
        self._active_action_id: str | None = None
        self._active_fire_id: str | None = None
        self._goal_handle = None
        self._control_mode = 'NONE'
        control_qos = QoSProfile(depth=1)
        control_qos.reliability = ReliabilityPolicy.RELIABLE
        control_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, '/vla/control_mode', self._control_mode_callback, control_qos
        )
        self.get_logger().info(
            'VLA spray bridge ready: /vla/spray_command -> /suppress_fire'
        )

    def _on_command(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            action_id = str(payload['action_id'])
            fire_id = str(payload['fire_id'])
            if payload.get('command') != 'SPRAY':
                raise ValueError('command must be SPRAY')
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f'invalid VLA spray command: {exc}')
            return

        if self._control_mode != 'VLA':
            self._publish_result(
                action_id, fire_id, 'FAILED', 'CONTROL_MODE_MISMATCH'
            )
            return

        if self._active_action_id is not None:
            self.get_logger().warning('spray action already active; command ignored')
            return
        if not self._client.wait_for_server(timeout_sec=1.0):
            self._publish_result(
                action_id, fire_id, 'FAILED', 'suppress_fire action unavailable'
            )
            return

        goal = SuppressFire.Goal()
        goal.max_attempts = max(1, min(255, self._max_attempts))
        self._active_action_id = action_id
        self._active_fire_id = fire_id
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _control_mode_callback(self, msg: String) -> None:
        self._control_mode = msg.data.strip().upper()

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finish('FAILED', f'suppress_fire send failed: {exc}')
            return
        if not goal_handle.accepted:
            self._finish('FAILED', 'suppress_fire goal rejected')
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            if wrapped.status == GoalStatus.STATUS_CANCELED:
                status = 'CANCELED'
            elif wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                # The action owns physical attempt completion; WorldModel owns
                # the subsequent fire-disappearance verification.
                status = 'SUCCEEDED'
            else:
                status = 'ABORTED'
            message = str(result.message)
        except Exception as exc:
            status = 'FAILED'
            message = f'suppress_fire result failed: {exc}'
        self._finish(status, message)

    def _on_cancel(self, msg: String) -> None:
        if self._goal_handle is None or self._active_action_id is None:
            return
        try:
            requested = str(json.loads(msg.data)['action_id'])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        if requested == self._active_action_id:
            self._goal_handle.cancel_goal_async()

    def _finish(self, status: str, message: str) -> None:
        if self._active_action_id is not None and self._active_fire_id is not None:
            self._publish_result(
                self._active_action_id,
                self._active_fire_id,
                status,
                message,
            )
        self._active_action_id = None
        self._active_fire_id = None
        self._goal_handle = None

    def _publish_result(
        self,
        action_id: str,
        fire_id: str,
        status: str,
        message: str,
    ) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                'action_id': action_id,
                'fire_id': fire_id,
                'status': status,
                'message': message,
            },
            ensure_ascii=False,
        )
        self._result_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VLASprayBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
