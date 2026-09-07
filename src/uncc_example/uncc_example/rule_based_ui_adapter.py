"""ROS adapter exposing the Rule-based runtime to Firefighter UI."""

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_action_status_default,
)

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String, UInt16
from std_srvs.srv import Trigger

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
        self._start_mission_client = self.create_client(
            Trigger, '/state_manager/start_mission'
        )
        self._stop_mission_client = self.create_client(
            Trigger, '/state_manager/stop_mission'
        )
        self._reset_mission_client = self.create_client(
            Trigger, '/state_manager/reset_mission'
        )

        # state_manager 가 transient_local로 발행하므로(늦게 붙어도 마지막
        # 값을 받기 위해), 여기도 durability를 맞춰야 실제로 그 값을 받는다
        # — volatile로 두면 호환은 되지만 late-joiner 샘플은 안 온다.
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String, '/mission/state', self._mission_state_callback, latched_qos
        )
        self.create_subscription(
            Bool, '/mission/manual_stop', self._manual_stop_callback, 10
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

    def _manual_stop_callback(self, msg):
        self.status.manual_stop = msg.data

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
        client = {
            'START': self._start_mission_client,
            'STOP': self._stop_mission_client,
            'RESET': self._reset_mission_client,
        }[command['command']]
        self._call_mission_trigger(client)
        self.status.last_command = {
            **command,
            'status': 'ACCEPTED',
        }
        self.status.blocked_reason = ''

    def _call_mission_trigger(self, client):
        if not client.service_is_ready():
            self.get_logger().warning(
                f'{client.srv_name} 서비스가 아직 준비되지 않았습니다.'
            )
            return

        def _on_response(future):
            try:
                result = future.result()
            except Exception as exc:  # noqa: BLE001 - 서비스 호출 실패를 그냥 로깅만 함
                self.get_logger().warning(f'{client.srv_name} 호출 실패: {exc}')
                return
            if not result.success:
                self.get_logger().warning(
                    f'{client.srv_name} 실패: {result.message}'
                )

        client.call_async(Trigger.Request()).add_done_callback(_on_response)

    def _publish_status(self):
        # discovery 가 끝나기 전에 눌리면 서비스 호출이 조용히 드롭되므로
        # (경고 로그만 남고 UI엔 아무 표시가 없음), UI가 버튼을 비활성화할
        # 수 있게 매번 최신 준비 상태를 스냅샷에 실어 보낸다.
        self.status.mission_ready = (
            self._start_mission_client.service_is_ready()
            and self._stop_mission_client.service_is_ready()
            and self._reset_mission_client.service_is_ready()
        )

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
