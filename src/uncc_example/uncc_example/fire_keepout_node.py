import json

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo

from .log_utils import make_event_logger


# fire/person 은 state_manager 가 한 번 기록하면 위치를 다시 안 바꾸므로
# (재감지는 dedup 되어 무시됨), round() 로 만든 좌표 키가 그대로 안정적인
# 식별자가 된다 — 위치 변경 추적/갱신 로직이 따로 필요 없다.
_COORD_PRECISION = 2

class FireKeepoutNode(Node):
    """
    /mission/found_targets(state_manager 가 발행하는 fire/person 발견
    기록 전체 스냅샷)를 구독해서, 각 대상 주변 반경을 Nav2 KeepoutFilter
    용 OccupancyGrid(/fire_keepout_mask)로 변환해 발행한다.

    - fire 는 LiDAR보다 낮아서 기존 costmap obstacle_layer 가 못 보므로
      별도 semantic keepout 이 필요하다.
    - person(테스트 중엔 마네킹)도 로봇이 그 위로 올라타면 안 되므로
      마찬가지로 keepout 대상이다.
    - 한 번 등록된 대상은 상태(진압 완료 등)가 바뀌어도 계속 keepout
      으로 유지한다 — 화재 받침대/컵 같은 실제 물체가 꺼진 뒤에도
      바닥에 남아있고 LiDAR 는 여전히 못 보기 때문.
    - 감지 즉시(fire_unvisited/person_unconfirmed 단계부터) 등록한다 —
      방문/확인이 끝난 뒤에 등록하면, 그 시점엔 로봇이 이미 바로 옆에
      서 있어서 자기가 만든 keepout 안에 갇히는 문제가 있었다.
    """

    def __init__(self):
        super().__init__('fire_keepout_node')

        self._event_logger = make_event_logger(self)

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('targets_topic', '/mission/found_targets')
        self.declare_parameter('mask_topic', '/fire_keepout_mask')
        self.declare_parameter('filter_info_topic', '/fire_keepout_mask_info')
        self.declare_parameter('circles_topic', '/fire_keepout_circles')
        self.declare_parameter('suppress_topic', '/fire_keepout_suppress')

        # keepout + footprint(0.195m) <= xy_goal_tolerance(0.32m) 맞춰 축소.
        self.declare_parameter('fire_keepout_radius', 0.05)
        self.declare_parameter('person_keepout_radius', 0.10)

        self.fire_keepout_radius = (
            self.get_parameter('fire_keepout_radius').value
        )
        self.person_keepout_radius = (
            self.get_parameter('person_keepout_radius').value
        )

        # -----------------------------
        # State
        # -----------------------------
        self._map_info = None  # (resolution, width, height, origin_x, origin_y)
        # key -> radius. key = (kind, round(x, _COORD_PRECISION), round(y, ...))
        self._tracked = {}
        # mission_executor 가 keepout 이탈(Spin/DriveOnHeading) 중에만
        # 잠깐 mask 에서 빼달라고 요청한 대상들 — _tracked 에서 지우는 게
        # 아니라 mask/circles 발행에서만 제외한다.
        self._suppressed = set()

        # -----------------------------
        # QoS
        # -----------------------------
        # map_server/costmap static_layer 와 동일한 관례 — 최신 값 1개만
        # 유지하고, 늦게 붙는 구독자(새 costmap 등)에게도 그대로 전달.
        transient_local_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -----------------------------
        # Subscriptions
        # -----------------------------
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self._map_callback,
            transient_local_qos,
        )

        self.create_subscription(
            String,
            self.get_parameter('targets_topic').value,
            self._targets_callback,
            10,
        )

        self.create_subscription(
            String,
            self.get_parameter('suppress_topic').value,
            self._suppress_callback,
            10,
        )

        # -----------------------------
        # Publishers
        # -----------------------------
        self.mask_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('mask_topic').value,
            transient_local_qos,
        )

        self.filter_info_pub = self.create_publisher(
            CostmapFilterInfo,
            self.get_parameter('filter_info_topic').value,
            transient_local_qos,
        )

        # mission_executor 가 "지금 로봇이 어떤 keepout 원과 겹쳤는지"
        # 판단할 때 쓰는 원본 좌표/반경 목록 (mask 는 격자라 역산이 번거로움).
        self.circles_pub = self.create_publisher(
            String,
            self.get_parameter('circles_topic').value,
            transient_local_qos,
        )

    # =========================================================
    # /map — grid 크기/해상도 기준 확보용. 한 번만 받아도 충분하지만
    # (맵이 갱신되는 경우를 대비해) 새로 들어올 때마다 기존 keepout 을
    # 새 grid 기준으로 다시 그린다.
    # =========================================================

    def _map_callback(self, msg):

        new_info = {
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
        }

        # grid 크기/해상도/원점이 그대로면 마스크 내용도 그대로다. SLAM이
        # /map 을 계속(장애물 변화 없이도) 갱신하므로, 여기서 매번 재발행하면
        # KeepoutFilter 가 "새 마스크 도착"으로 오인해 매번 로그를 찍는다
        # — _targets_callback 이 실제로 대상이 바뀔 때 이미 재발행한다.
        if self._tracked and new_info == self._map_info:
            self._map_info = new_info
            return

        self._map_info = new_info

        if self._tracked:
            self._publish_mask()

    # =========================================================
    # /mission/found_targets — fire/person 발견 스냅샷. 새 대상이
    # 생겼을 때만(이미 기록된 대상은 위치가 안 바뀌므로) mask 를
    # 재생성한다.
    # =========================================================

    def _targets_callback(self, msg):

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid found_targets JSON: {e}')
            return

        changed = False

        for target in payload.get('targets', []):

            kind = self._keepout_kind(target.get('type', ''))

            if kind is None:
                continue

            key = (
                kind,
                round(target['x'], _COORD_PRECISION),
                round(target['y'], _COORD_PRECISION),
            )

            if key in self._tracked:
                continue

            radius = (
                self.fire_keepout_radius if kind == 'fire'
                else self.person_keepout_radius
            )
            self._tracked[key] = radius
            changed = True

            self._event_logger.info(
                f'keepout 등록: {kind} '
                f'({key[1]:.2f}, {key[2]:.2f}) radius={radius:.2f}m'
            )

        if changed and self._map_info is not None:
            self._publish_mask()

    # =========================================================
    # /fire_keepout_suppress — mission_executor 가 keepout 이탈
    # (Spin/DriveOnHeading) 중에만 특정 원을 mask 에서 빼달라고 요청.
    # Spin 자체가 로컬 costmap 기준으로 스윕 충돌검사를 하기 때문에,
    # 로봇 발판이 이미 걸친 keepout 을 그대로 두면 회전조차 못 한다.
    # =========================================================

    def _suppress_callback(self, msg):

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid fire_keepout_suppress JSON: {e}')
            return

        x = payload.get('x')
        y = payload.get('y')

        key = next(
            (k for k in self._tracked if k[1] == x and k[2] == y),
            None,
        )

        if key is None:
            return

        if payload.get('suppress', True):
            self._suppressed.add(key)
        else:
            self._suppressed.discard(key)

        if self._map_info is not None:
            self._publish_mask()

    @staticmethod
    def _keepout_kind(category):
        """state_manager 가 붙이는 7분류(fire_unvisited 등)로 fire/person
        을 가른다."""

        if category.startswith('fire_'):
            return 'fire'

        if category.startswith('person_'):
            return 'person'

        return None

    # =========================================================
    # Mask 생성/발행
    # =========================================================

    def _publish_mask(self):

        info = self._map_info
        resolution = info['resolution']
        width = info['width']
        height = info['height']
        origin_x = info['origin_x']
        origin_y = info['origin_y']

        grid = [0] * (width * height)

        for (kind, x, y), radius in self._tracked.items():

            if (kind, x, y) in self._suppressed:
                continue

            center_col = int((x - origin_x) / resolution)
            center_row = int((y - origin_y) / resolution)
            radius_cells = int(radius / resolution) + 1

            for row in range(
                max(0, center_row - radius_cells),
                min(height, center_row + radius_cells + 1),
            ):
                for col in range(
                    max(0, center_col - radius_cells),
                    min(width, center_col + radius_cells + 1),
                ):
                    dx = col - center_col
                    dy = row - center_row
                    if dx * dx + dy * dy <= radius_cells * radius_cells:
                        grid[row * width + col] = 100

        stamp = self.get_clock().now().to_msg()

        mask_msg = OccupancyGrid()
        mask_msg.header.frame_id = 'map'
        mask_msg.header.stamp = stamp
        mask_msg.info.resolution = resolution
        mask_msg.info.width = width
        mask_msg.info.height = height
        mask_msg.info.origin.position.x = origin_x
        mask_msg.info.origin.position.y = origin_y
        mask_msg.data = grid

        self.mask_pub.publish(mask_msg)

        filter_info_msg = CostmapFilterInfo()
        filter_info_msg.header.frame_id = 'map'
        filter_info_msg.header.stamp = stamp
        filter_info_msg.type = 0  # keepout/lanes filter
        filter_info_msg.filter_mask_topic = (
            self.get_parameter('mask_topic').value
        )
        filter_info_msg.base = 0.0
        filter_info_msg.multiplier = 1.0

        self.filter_info_pub.publish(filter_info_msg)

        circles_msg = String()
        circles_msg.data = json.dumps([
            {'x': x, 'y': y, 'radius': radius}
            for (kind, x, y), radius in self._tracked.items()
            if (kind, x, y) not in self._suppressed
        ])
        self.circles_pub.publish(circles_msg)


def main(args=None):

    rclpy.init(args=args)

    node = FireKeepoutNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
