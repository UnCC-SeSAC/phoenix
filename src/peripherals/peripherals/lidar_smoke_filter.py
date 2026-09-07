#!/usr/bin/env python3
"""LD19 LaserScan에서 강도(intensity)가 낮은 점을 걸러낸다.

연기/안개 입자에 부딪혀 돌아온 레이저 반사는 단단한 장애물보다
반사 강도가 현저히 낮다. intensity가 threshold 미만인 점은
range를 inf로 지워서(표준 "무효/최대거리 밖" 표현) SLAM/costmap이
장애물로 보지 않게 한다.

LD19 원본 토픽 이름을 그대로 쓰면 다른 노드(slam_toolbox,
rf2o_laser_odometry 등)를 하나도 안 건드리고 끼워넣을 수 있다 —
드라이버가 발행하는 "원본"을 input_topic으로 받고, 지금까지 다들
구독하던 이름(output_topic, 기본 scan_raw)으로 다시 내보낸다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarSmokeFilter(Node):
    def __init__(self):
        super().__init__('lidar_smoke_filter')

        self.declare_parameter('input_topic', 'scan_raw_unfiltered')
        self.declare_parameter('output_topic', 'scan_raw')
        # LD19 intensity는 0~255 범위(추정치, 실측으로 조정 필요).
        self.declare_parameter('intensity_threshold', 200.0)
        self.declare_parameter('stats_period_sec', 5.0)

        self._threshold = float(self.get_parameter('intensity_threshold').value)
        self._stats_period = float(self.get_parameter('stats_period_sec').value)

        self._pub = self.create_publisher(
            LaserScan,
            str(self.get_parameter('output_topic').value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter('input_topic').value),
            self._on_scan,
            qos_profile_sensor_data,
        )

        self._n_scans = 0
        self._n_points = 0
        self._n_dropped = 0
        self._last_report = self.get_clock().now()

        self.get_logger().info(
            f'[smoke-filter] {self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value}, '
            f'intensity_threshold={self._threshold:g}'
        )

    def _on_scan(self, msg: LaserScan) -> None:
        if len(msg.intensities) != len(msg.ranges):
            # intensity 정보가 없는(길이 불일치) 드라이버라면 걸러낼 방법이
            # 없다 — 필터 없이 그대로 통과시켜서 스캔 자체가 끊기지 않게 한다.
            self._pub.publish(msg)
            return

        ranges = list(msg.ranges)
        dropped = 0
        for i, intensity in enumerate(msg.intensities):
            if intensity < self._threshold:
                ranges[i] = float('inf')
                dropped += 1
        msg.ranges = ranges
        self._pub.publish(msg)

        self._n_scans += 1
        self._n_points += len(ranges)
        self._n_dropped += dropped
        self._report()

    def _report(self) -> None:
        if self._stats_period <= 0:
            return
        now = self.get_clock().now()
        if (now - self._last_report).nanoseconds < self._stats_period * 1e9:
            return
        pct = (100.0 * self._n_dropped / self._n_points) if self._n_points else 0.0
        self.get_logger().info(
            f'[smoke-filter] scans={self._n_scans} '
            f'dropped={self._n_dropped}/{self._n_points} ({pct:.1f}%)'
        )
        self._n_scans = self._n_points = self._n_dropped = 0
        self._last_report = now


def main(args=None):
    rclpy.init(args=args)
    node = LidarSmokeFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
