#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rf2o_laser_odometry가 covariance를 0으로 채워서 발행하는 문제를 보완하는 릴레이.
robot_localization(EKF)은 covariance=0인 성분을 fusion에서 제외하기 때문에,
pose/twist 값은 그대로 두고 covariance만 실측 기반 고정값으로 채워 재발행한다.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# 6x6 covariance 행렬의 (row, col) 위치 -> 값. 순서: x,y,z,roll,pitch,yaw
POSE_COV = {
    (0, 0): 0.02,   # x
    (1, 1): 0.02,   # y
    (5, 5): 0.05,   # yaw
}
TWIST_COV = {
    (0, 0): 0.02,   # vx
    (5, 5): 0.05,   # vyaw
}


class Rf2oCovarianceRelay(Node):
    def __init__(self):
        super().__init__('rf2o_covariance_relay')
        self.pub = self.create_publisher(Odometry, 'odom_rf2o_fixed', 10)
        self.sub = self.create_subscription(
            Odometry, 'odom_rf2o', self.callback, 10
        )

    def callback(self, msg: Odometry):
        pose_cov = list(msg.pose.covariance)
        twist_cov = list(msg.twist.covariance)
        for (r, c), v in POSE_COV.items():
            pose_cov[r * 6 + c] = v
        for (r, c), v in TWIST_COV.items():
            twist_cov[r * 6 + c] = v
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Rf2oCovarianceRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()