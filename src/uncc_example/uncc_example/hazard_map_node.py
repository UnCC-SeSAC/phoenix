import math
import time
from dataclasses import dataclass
from typing import List

import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


@dataclass
class HazardPoint:
    x: float
    y: float
    stamp: float


class HazardMapNode(Node):
    """
    Builds /hazard_map with the same geometry as /map.

    Input:
      /hazard_points (geometry_msgs/PoseArray, expected in map frame)

    Value convention:
       0   safe
       1-99 risk
       100 lethal/high risk
    """

    def __init__(self):
        super().__init__('hazard_map_node')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('hazard_points_topic', '/hazard_points')
        self.declare_parameter('hazard_map_topic', '/hazard_map')
        self.declare_parameter('hazard_radius', 1.0)
        self.declare_parameter('lethal_radius', 0.25)
        self.declare_parameter('hazard_ttl', 60.0)
        self.declare_parameter('publish_frequency', 1.0)

        self.map_topic = self.get_parameter('map_topic').value
        self.hazard_points_topic = self.get_parameter(
            'hazard_points_topic'
        ).value
        self.hazard_map_topic = self.get_parameter(
            'hazard_map_topic'
        ).value

        self.hazard_radius = float(
            self.get_parameter('hazard_radius').value
        )
        self.lethal_radius = float(
            self.get_parameter('lethal_radius').value
        )
        self.hazard_ttl = float(
            self.get_parameter('hazard_ttl').value
        )
        publish_frequency = max(
            0.2,
            float(self.get_parameter('publish_frequency').value),
        )

        self.latest_map = None
        self.hazards: List[HazardPoint] = []

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            transient_qos,
        )

        self.hazard_sub = self.create_subscription(
            PoseArray,
            self.hazard_points_topic,
            self._hazard_callback,
            10,
        )

        self.publisher = self.create_publisher(
            OccupancyGrid,
            self.hazard_map_topic,
            transient_qos,
        )

        self.timer = self.create_timer(
            1.0 / publish_frequency,
            self._publish,
        )

        self.get_logger().info(
            f'HazardMapNode: {self.hazard_points_topic} -> '
            f'{self.hazard_map_topic}'
        )

    def _map_callback(self, msg):
        self.latest_map = msg

    def _hazard_callback(self, msg: PoseArray):
        if self.latest_map is None:
            return

        map_frame = self.latest_map.header.frame_id or 'map'
        if msg.header.frame_id and msg.header.frame_id != map_frame:
            self.get_logger().warn(
                f'Ignoring hazard_points frame={msg.header.frame_id}; '
                f'expected {map_frame}. Transform detections to map first.',
                throttle_duration_sec=5.0,
            )
            return

        now = time.monotonic()
        for pose in msg.poses:
            self.hazards.append(
                HazardPoint(
                    x=float(pose.position.x),
                    y=float(pose.position.y),
                    stamp=now,
                )
            )

    def _purge(self):
        if self.hazard_ttl <= 0:
            return

        now = time.monotonic()
        self.hazards = [
            h
            for h in self.hazards
            if now - h.stamp <= self.hazard_ttl
        ]

    def _publish(self):
        if self.latest_map is None:
            return

        self._purge()

        src = self.latest_map
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = src.header.frame_id
        out.info = src.info

        width = int(src.info.width)
        height = int(src.info.height)
        resolution = float(src.info.resolution)
        ox = float(src.info.origin.position.x)
        oy = float(src.info.origin.position.y)

        data = [0] * (width * height)

        radius_cells = max(
            1,
            int(math.ceil(self.hazard_radius / resolution)),
        )

        for hazard in self.hazards:
            cx = int(math.floor((hazard.x - ox) / resolution))
            cy = int(math.floor((hazard.y - oy) / resolution))

            for my in range(cy - radius_cells, cy + radius_cells + 1):
                if my < 0 or my >= height:
                    continue

                for mx in range(cx - radius_cells, cx + radius_cells + 1):
                    if mx < 0 or mx >= width:
                        continue

                    wx = ox + (mx + 0.5) * resolution
                    wy = oy + (my + 0.5) * resolution
                    d = math.hypot(wx - hazard.x, wy - hazard.y)

                    if d > self.hazard_radius:
                        continue

                    if d <= self.lethal_radius:
                        risk = 100
                    else:
                        denom = max(
                            1e-6,
                            self.hazard_radius - self.lethal_radius,
                        )
                        ratio = 1.0 - (
                            (d - self.lethal_radius) / denom
                        )
                        risk = int(max(1.0, min(99.0, 99.0 * ratio)))

                    idx = my * width + mx
                    data[idx] = max(data[idx], risk)

        out.data = data
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = HazardMapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
