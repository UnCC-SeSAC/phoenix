#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rf2o_laser_odometry 출력을 EKF 가 쓸 수 있게 손보는 릴레이. 두 가지를 고친다.

1. covariance 가 전부 0이다
   rf2o 는 pose/twist covariance 36칸을 모두 0으로 채워 발행한다(live09 실측 확인).
   실측 기반 고정값으로 채워 재발행한다.

2. ★ 프레임이 180도 돌아가 있다 (2026-09-01 확인)
   이 로봇은 라이다가 뒤를 보고 장착돼 있고 URDF 가 그걸 정확히 선언한다
   (mentorpi_description/urdf/lidar.urdf.xacro:8, rpy="0 0 pi"). 그런데 rf2o 는
   base_frame_id 를 줘도 자기 출력을 **레이저 프레임 그대로** 발행한다.

   실측 근거:
     - 지령 vx>0 (전진) 일 때 rf2o vx 중앙 -0.12  → x 부호 반대
     - odom_raw 와 rf2o 의 총 이동 벡터가 148~159도 차이 (경로 길이는 비슷)
     - 반면 회전은 자이로와 상관 +0.60~0.83 → yaw 부호는 정상
   x,y 만 뒤집히고 yaw 는 멀쩡한 것이 z축 180도 회전의 특징이다(거울이면 yaw 도 뒤집힌다).

   이걸 안 고치면 EKF 가 휠 오도메트리와 정반대 방향의 위치를 융합한다
   (ekf.yaml 의 odom1 이 이 토픽의 pose x,y 를 쓴다).

   여기서 180도 회전을 적용한다: x,y 와 vx,vy 의 부호를 뒤집고 yaw 는 그대로 둔다.
   rf2o 를 고치는 게 근본 해결이지만 서드파티 패키지라 여기서 보정한다.
"""
import math

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
        # --- 180도 회전 보정 ---
        # z축 180도 회전은 (x, y) -> (-x, -y) 이고 yaw 는 그대로다.
        msg.pose.pose.position.x = -msg.pose.pose.position.x
        msg.pose.pose.position.y = -msg.pose.pose.position.y
        msg.twist.twist.linear.x = -msg.twist.twist.linear.x
        msg.twist.twist.linear.y = -msg.twist.twist.linear.y
        # 자세(yaw)는 건드리지 않는다 — 실측에서 자이로와 부호가 일치했다.

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