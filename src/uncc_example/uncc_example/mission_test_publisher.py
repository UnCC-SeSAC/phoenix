import argparse
import json

import rclpy

from rclpy.node import Node

from std_msgs.msg import String, UInt16
from visualization_msgs.msg import Marker


class MissionTestPublisher(Node):
    """
    로봇/카메라 없이 state_manager 를 테스트하기 위한 CLI 발행기.

    ros2 run uncc_example mission_test --fire 1.0 2.0
    ros2 run uncc_example mission_test --person 3.0 0.5
    ros2 run uncc_example mission_test --battery 6000
    ros2 run uncc_example mission_test --frontier 5.0 5.0

    옵션은 한 번에 여러 개를 같이 줘도 된다.
    """

    def __init__(self, args):
        super().__init__('mission_test_publisher')

        self.args = args
        self.done = False

        self.detection_pub = self.create_publisher(
            String,
            '/vision/detections',
            10,
        )

        self.battery_pub = self.create_publisher(
            UInt16,
            '/ros_robot_controller/battery',
            1,
        )

        self.frontier_pub = self.create_publisher(
            Marker,
            '/exploration/best_frontier',
            10,
        )

        self.timer = self.create_timer(0.5, self._publish_once)

    def _publish_once(self):

        if self.done:
            return

        if self.args.fire is not None:
            self._publish_detection('fire', *self.args.fire)

        if self.args.person is not None:
            self._publish_detection('person', *self.args.person)

        if self.args.battery is not None:
            self._publish_battery(self.args.battery)

        if self.args.frontier is not None:
            self._publish_frontier(*self.args.frontier)

        self.done = True

    def _publish_detection(self, class_name, x, y):

        msg = String()
        msg.data = json.dumps({
            'class': class_name,
            'x': x,
            'y': y,
            'frame_id': 'map',
        })
        self.detection_pub.publish(msg)

        self.get_logger().info(
            f'Published {class_name} detection at ({x:.2f}, {y:.2f})'
        )

    def _publish_battery(self, value):

        msg = UInt16()
        msg.data = value
        self.battery_pub.publish(msg)

        self.get_logger().info(
            f'Published battery value {value}'
        )

    def _publish_frontier(self, x, y):

        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0

        self.frontier_pub.publish(msg)

        self.get_logger().info(
            f'Published frontier target at ({x:.2f}, {y:.2f})'
        )


def main(args=None):

    parser = argparse.ArgumentParser()

    parser.add_argument('--fire', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--person', nargs=2, type=float, metavar=('X', 'Y'))
    parser.add_argument('--battery', type=int)
    parser.add_argument('--frontier', nargs=2, type=float, metavar=('X', 'Y'))

    known, ros_args = parser.parse_known_args(args=args)

    if (
        known.fire is None
        and known.person is None
        and known.battery is None
        and known.frontier is None
    ):
        parser.error(
            '--fire, --person, --battery, --frontier 중 '
            '최소 하나는 지정해야 함'
        )

    rclpy.init(args=ros_args)
    node = MissionTestPublisher(known)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        # DDS 가 메시지를 실제로 내보낼 시간을 조금 준다
        rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
