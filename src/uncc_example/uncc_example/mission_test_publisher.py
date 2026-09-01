import argparse
import json

import rclpy

from rclpy.node import Node

from std_msgs.msg import String, UInt16
from visualization_msgs.msg import Marker


class MissionTestPublisher(Node):
    """
    로봇/카메라 없이 state_manager 를 테스트하기 위한 CLI 발행기.

    state_manager 는 start_mission 서비스가 호출되기 전까지 STANDBY 로
    대기하므로, 상태 전이까지 보려면 먼저 아래를 호출해야 한다:
    ros2 service call /state_manager/start_mission std_srvs/srv/Trigger "{}"

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

        # vision_detector 가 실제로 보내는 형식(프레임 하나에 감지
        # 여러 개를 묶은 배치)을 흉내내려고, fire/person 을 따로따로
        # 안 보내고 이번 호출에서 지정된 것들을 한 메시지로 묶는다
        # — state_manager 의 짝짓기(거리 기반)를 제대로 테스트하려면
        # 같은 배치 안에 있어야 하기 때문.
        detections = []

        if self.args.fire is not None:
            detections.append(self._make_detection('fire', *self.args.fire))

        if self.args.person is not None:
            detections.append(
                self._make_detection('person', *self.args.person)
            )

        if detections:
            self._publish_detections(detections)

        if self.args.battery is not None:
            self._publish_battery(self.args.battery)

        if self.args.frontier is not None:
            self._publish_frontier(*self.args.frontier)

        self.done = True

    def _make_detection(self, class_name, x, y):
        return {'class': class_name, 'x': x, 'y': y}

    def _publish_detections(self, detections):

        msg = String()
        msg.data = json.dumps({
            'frame_id': 'map',
            'detections': detections,
        })
        self.detection_pub.publish(msg)

        for detection in detections:
            self.get_logger().info(
                f"Published {detection['class']} detection at "
                f"({detection['x']:.2f}, {detection['y']:.2f})"
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
