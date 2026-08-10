import argparse

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node


class HazardTestPublisher(Node):
    def __init__(self, x: float, y: float):
        super().__init__('hazard_test_publisher')
        self.publisher = self.create_publisher(
            PoseArray,
            '/hazard_points',
            10,
        )
        self.x = float(x)
        self.y = float(y)
        self.timer = self.create_timer(0.5, self._publish_once)
        self.done = False

    def _publish_once(self):
        if self.done:
            return

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.orientation.w = 1.0

        msg.poses.append(pose)
        self.publisher.publish(msg)

        self.get_logger().info(
            f'Published test hazard at ({self.x:.2f}, {self.y:.2f})'
        )
        self.done = True


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--x', type=float, default=1.0)
    parser.add_argument('--y', type=float, default=0.0)
    known, ros_args = parser.parse_known_args(args=args)

    rclpy.init(args=ros_args)
    node = HazardTestPublisher(known.x, known.y)

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        # One extra spin gives DDS time to send the sample.
        rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
