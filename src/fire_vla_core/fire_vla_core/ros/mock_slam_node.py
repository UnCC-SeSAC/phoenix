"""하드웨어 없이 SLAM 화면을 시연하기 위한 목업 지도/TF 발행 노드.

정답 지도를 통째로 내지 않고 **로봇 주변부터 점진적으로 밝힙니다.** 완성된
지도를 처음부터 띄우면 "SLAM이 도는 것"과 "지도 파일을 읽은 것"이 화면에서
구분되지 않고, 그러면 이 목업으로는 UI가 실시간 갱신을 제대로 처리하는지
확인할 수 없습니다.

  발행  /map  (nav_msgs/OccupancyGrid, TRANSIENT_LOCAL)
  TF    map -> odom -> base_footprint

★ 실기 대체품이 아닙니다. 시연·회귀 확인 전용이고, 실제 slam_toolbox와
  같은 토픽·프레임·QoS를 쓰는 것이 유일한 목적입니다.
"""

from __future__ import annotations

import math
import time

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from tf2_ros import TransformBroadcaster
except ImportError:
    rclpy = None
    Node = object
    TransformStamped = None
    OccupancyGrid = None
    TransformBroadcaster = None
    QoSProfile = None
    HistoryPolicy = ReliabilityPolicy = DurabilityPolicy = None


_WALL = 100
_FREE = 0
_UNKNOWN = -1


def build_room(width: int, height: int) -> list[int]:
    """외벽 + 기둥 + 부분 칸막이의 '정답' 지도.

    구조물은 로봇의 원형 경로에서 비켜 둡니다 — 목업이라도 로봇이 벽을
    통과하면 좌표 버그와 구분이 안 됩니다.
    """
    grid = [_FREE] * (width * height)
    for x in range(width):
        grid[x] = _WALL
        grid[(height - 1) * width + x] = _WALL
    for y in range(height):
        grid[y * width] = _WALL
        grid[y * width + width - 1] = _WALL

    partition_x = int(width * 0.85)
    for y in range(int(height * 0.1), int(height * 0.6)):
        grid[y * width + partition_x] = _WALL

    for ratio_x, ratio_y in ((0.15, 0.20), (0.20, 0.80)):
        cx, cy = int(width * ratio_x), int(height * ratio_y)
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                x, y = cx + dx, cy + dy
                if 0 <= x < width and 0 <= y < height:
                    grid[y * width + x] = _WALL
    return grid


def reveal_disc(truth, known, width, height, center_x, center_y, radius) -> int:
    """로봇 주변 반경 radius 셀을 truth에서 known으로 옮깁니다.

    반환값은 이번에 새로 밝혀진 셀 수입니다. 0이 계속 나오면 탐색이 더는
    진행되지 않는 것이고, 화면에서는 지도가 멈춘 것으로 보입니다.
    """
    revealed = 0
    radius_squared = radius * radius
    for y in range(max(0, center_y - radius), min(height, center_y + radius + 1)):
        for x in range(max(0, center_x - radius), min(width, center_x + radius + 1)):
            dx, dy = x - center_x, y - center_y
            if dx * dx + dy * dy > radius_squared:
                continue
            index = y * width + x
            if known[index] != truth[index]:
                known[index] = truth[index]
                revealed += 1
    return revealed


def circle_pose(elapsed, radius, period) -> tuple[float, float, float]:
    """반경 radius 원을 period초에 한 바퀴. yaw는 진행 방향(접선)."""
    if period <= 0:
        return (radius, 0.0, math.pi / 2.0)
    angle = 2.0 * math.pi * (elapsed / period)
    return (
        radius * math.cos(angle),
        radius * math.sin(angle),
        angle + math.pi / 2.0,
    )


def world_to_cell(x, y, origin_x, origin_y, resolution) -> tuple[int, int]:
    return (
        int(math.floor((x - origin_x) / resolution)),
        int(math.floor((y - origin_y) / resolution)),
    )


def quaternion_from_yaw(yaw) -> tuple[float, float, float, float]:
    """(x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MockSlamNode(Node):
    def __init__(self) -> None:
        super().__init__("vla_mock_slam")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("width_m", 8.0)
        self.declare_parameter("height_m", 6.0)
        self.declare_parameter("sensor_range_m", 1.4)
        self.declare_parameter("path_radius_m", 2.0)
        self.declare_parameter("path_period_sec", 40.0)
        self.declare_parameter("map_period_sec", 0.5)
        self.declare_parameter("tf_period_sec", 0.05)

        p = self.get_parameter
        self._resolution = float(p("resolution").value)
        self._width = max(1, int(float(p("width_m").value) / self._resolution))
        self._height = max(1, int(float(p("height_m").value) / self._resolution))
        # 원점을 방 한가운데로 둡니다 — world (0,0)이 화면 정중앙이라
        # 투영식이 틀렸는지 눈으로 바로 알 수 있습니다.
        self._origin_x = -float(p("width_m").value) / 2.0
        self._origin_y = -float(p("height_m").value) / 2.0
        self._sensor_cells = max(1, int(
            float(p("sensor_range_m").value) / self._resolution))
        self._path_radius = float(p("path_radius_m").value)
        self._path_period = float(p("path_period_sec").value)
        self._map_frame = str(p("map_frame").value)
        self._odom_frame = str(p("odom_frame").value)
        self._base_frame = str(p("base_frame").value)

        self._truth = build_room(self._width, self._height)
        self._known = [_UNKNOWN] * (self._width * self._height)
        self._start = time.monotonic()

        # ★ 실제 slam_toolbox와 같은 latched QoS. 여기서 어긋나면 목업에서는
        #   되는데 실기에서 화면이 비는 상황이 생깁니다.
        self._map_pub = self.create_publisher(
            OccupancyGrid,
            str(p("map_topic").value),
            QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._tf = TransformBroadcaster(self)
        self.create_timer(float(p("tf_period_sec").value), self._publish_tf)
        self.create_timer(float(p("map_period_sec").value), self._publish_map)
        self.get_logger().info(
            f"[목업 SLAM] {self._width}x{self._height} cells @ "
            f"{self._resolution}m — 실기 대체품이 아닙니다"
        )

    def _pose(self) -> tuple[float, float, float]:
        return circle_pose(
            time.monotonic() - self._start, self._path_radius, self._path_period
        )

    def _publish_tf(self) -> None:
        x, y, yaw = self._pose()
        now = self.get_clock().now().to_msg()
        # map -> odom 은 항등. 실기에서는 SLAM이 보정하는 부분입니다.
        self._tf.sendTransform([
            self._transform(now, self._map_frame, self._odom_frame, 0.0, 0.0, 0.0),
            self._transform(now, self._odom_frame, self._base_frame, x, y, yaw),
        ])

    def _transform(self, stamp, parent, child, x, y, yaw):
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = float(x)
        message.transform.translation.y = float(y)
        message.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        message.transform.rotation.x = qx
        message.transform.rotation.y = qy
        message.transform.rotation.z = qz
        message.transform.rotation.w = qw
        return message

    def _publish_map(self) -> None:
        x, y, _ = self._pose()
        cell_x, cell_y = world_to_cell(
            x, y, self._origin_x, self._origin_y, self._resolution
        )
        reveal_disc(
            self._truth, self._known, self._width, self._height,
            cell_x, cell_y, self._sensor_cells,
        )

        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_frame
        message.info.resolution = self._resolution
        message.info.width = self._width
        message.info.height = self._height
        message.info.origin.position.x = self._origin_x
        message.info.origin.position.y = self._origin_y
        message.info.origin.orientation.w = 1.0
        message.data = list(self._known)
        self._map_pub.publish(message)


def main(args=None) -> None:
    if rclpy is None:
        raise RuntimeError("ROS2 환경에서 실행해야 합니다.")
    rclpy.init(args=args)
    node = MockSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()