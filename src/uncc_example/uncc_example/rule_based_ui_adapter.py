"""ROS adapter exposing the Rule-based runtime to Firefighter UI."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, UInt16

from .rule_based_ui_contract import RuleBasedStatus, parse_mission_command


STATUS_LABELS = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'RUNNING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}
ACTIVE_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


def summarize_goal_status(msg):
    statuses = [entry.status for entry in msg.status_list]
    for status in statuses:
        if status in ACTIVE_STATUSES:
            return STATUS_LABELS[status]
    if not statuses:
        return 'IDLE'
    return STATUS_LABELS.get(statuses[-1], 'UNKNOWN')


class RuleBasedUIAdapter(Node):
    def __init__(self):
        super().__init__('rule_based_ui_adapter')
        self.declare_parameter('status_topic', '/rule_based/status')
        self.declare_parameter('mission_topic', '/rule_based/mission')
        self.declare_parameter('publish_period_sec', 0.5)

        self.status = RuleBasedStatus()
        self._last_mission_id = None
        self._status_pub = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self._enabled_pub = self.create_publisher(
            Bool,
            '/mission/enabled',
            10,
        )

        self.create_subscription(
            String, '/mission/state', self._mission_state_callback, 10
        )
        self.create_subscription(
            String, '/mission/target_type', self._target_type_callback, 10
        )
        self.create_subscription(
            PoseStamped,
            '/mission/current_target',
            self._target_callback,
            10,
        )
        self.create_subscription(
            String,
            '/mission/found_targets',
            self._found_targets_callback,
            10,
        )
        self.create_subscription(
            UInt16,
            '/ros_robot_controller/battery',
            self._battery_callback,
            10,
        )
        self.create_subscription(
            String,
            '/rule_based/exploration_state',
            self._exploration_callback,
            10,
        )
        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self._navigation_callback,
            qos_profile_action_status_default,
        )
        self.create_subscription(
            GoalStatusArray,
            '/suppress_fire/_action/status',
            self._suppression_callback,
            qos_profile_action_status_default,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mission_topic').value),
            self._mission_command_callback,
            10,
        )
        self.create_timer(
            float(self.get_parameter('publish_period_sec').value),
            self._publish_status,
        )

    def _mission_state_callback(self, msg):
        self.status.mission_state = msg.data

    def _target_type_callback(self, msg):
        self.status.target_type = msg.data

    def _target_callback(self, msg):
        position = msg.pose.position
        self.status.current_target = {
            'frame_id': msg.header.frame_id,
            'x': position.x,
            'y': position.y,
        }

    def _found_targets_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            targets = payload.get('targets')
            if not isinstance(targets, list):
                raise ValueError('targets list가 필요합니다.')
            self.status.found_targets = targets
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.status.blocked_reason = f'found_targets parsing failed: {exc}'
            self.get_logger().warning(self.status.blocked_reason)

    def _battery_callback(self, msg):
        self.status.battery_raw = int(msg.data)

    def _exploration_callback(self, msg):
        self.status.exploration_status = msg.data

    def _navigation_callback(self, msg):
        self.status.navigation_status = summarize_goal_status(msg)

    def _suppression_callback(self, msg):
        self.status.suppression_status = summarize_goal_status(msg)

    def _mission_command_callback(self, msg):
        try:
            command = parse_mission_command(msg.data)
        except ValueError as exc:
            self.status.blocked_reason = str(exc)
            self.get_logger().warning(self.status.blocked_reason)
            return

        if command['mission_id'] == self._last_mission_id:
            return

        self._last_mission_id = command['mission_id']
        enabled = command['command'] == 'START'
        self._enabled_pub.publish(Bool(data=enabled))
        self.status.last_command = {
            **command,
            'status': 'ACCEPTED',
        }
        self.status.blocked_reason = ''

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps(
            self.status.snapshot(),
            ensure_ascii=False,
            separators=(',', ':'),
        )
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RuleBasedUIAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
