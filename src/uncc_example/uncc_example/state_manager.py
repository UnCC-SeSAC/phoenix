import json
import math

import rclpy

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener, TransformException

from std_msgs.msg import Header, String, UInt16
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker


class StateManager(Node):

    # 화재/인명 정보가 전혀 없을 때: Frontier 가 알려주는 미탐사 위치로 이동
    EXPLORING = 'EXPLORING'
    # 대기중인 목적지 중 사람이 가장 가까움: 구조 접근
    PERSON_DETECTED = 'PERSON_DETECTED'
    # 대기중인 목적지 중 불이 가장 가까움: 진화 접근
    FIRE_DETECTED = 'FIRE_DETECTED'
    # 배터리가 임계치 이하: 다른 모든 목적지보다 우선하여 복귀
    RETURNING_TO_BASE = 'RETURNING_TO_BASE'

    def __init__(self):
        super().__init__('state_manager')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # 로봇 컨트롤러가 보내는 raw 배터리 값 기준 (기기에 맞게 튜닝 필요)
        self.declare_parameter('low_battery_threshold', 7000)

        # 이 반경 안의 같은 종류(fire/person) 감지는 하나의 목적지로 병합
        self.declare_parameter('target_merge_radius', 0.5)

        # 불과 사람이 동시에 감지됐을 때, 이 거리 이내로 붙어 있으면
        # 사람이 위험하다고 보고 불부터 끄고, 이보다 멀리 떨어져 있으면
        # 사람부터 먼저 확인한다
        self.declare_parameter('fire_person_proximity_threshold', 2.0)

        self.declare_parameter('state_check_period', 0.2)

        # 충전 도크 위치 (map frame). 실제 도크 좌표로 반드시 수정할 것
        self.declare_parameter('base_x', 0.0)
        self.declare_parameter('base_y', 0.0)
        self.declare_parameter('base_yaw', 0.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.low_battery_threshold = (
            self.get_parameter('low_battery_threshold').value
        )
        self.target_merge_radius = (
            self.get_parameter('target_merge_radius').value
        )
        self.fire_person_proximity_threshold = (
            self.get_parameter('fire_person_proximity_threshold').value
        )

        self.base_pose = self._build_base_pose()

        # -----------------------------
        # State
        # -----------------------------
        self.state = self.EXPLORING

        self.robot_x = None
        self.robot_y = None

        self.battery_raw = None

        self.frontier_target = None  # 최신 Frontier 추천 목적지 (PoseStamped)

        # fire/person 감지가 쌓이는 목적지 큐. 원소: {'type', 'pose'}
        # found_targets 와 원소(dict)를 공유한다 — 아직 처리 안 한
        # 것만 여기 들어있고, target_complete 로 여기서만 빠진다.
        self.target_queue = []
        self.active_target = None  # 현재 상태로 채택된 목적지 (완료 통보 대상)

        # 미션 시작부터 끝까지 유지되는 발견 기록 (완료된 것도 안 지움).
        # 신규 감지 중복 판단(dedup)과, 나중에 지도에 표시할 데이터로 쓴다.
        self.found_targets = []

        self._last_logged_state = None

        # -----------------------------
        # TF (로봇 현재 위치 확인용)
        # -----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------
        # Detections (vision_detector 가 YOLO + depth 를 map 좌표로
        # 변환해 JSON 으로 publish)
        # -----------------------------
        self.create_subscription(
            String,
            '/vision/detections',
            self.detection_callback,
            10,
        )

        # -----------------------------
        # Battery
        # -----------------------------
        self.create_subscription(
            UInt16,
            '/ros_robot_controller/battery',
            self.battery_callback,
            1,
        )

        # -----------------------------
        # Frontier exploration 이 추천하는 다음 목적지
        # -----------------------------
        self.create_subscription(
            Marker,
            '/exploration/best_frontier',
            self.frontier_callback,
            10,
        )

        # -----------------------------
        # Publishers (실제 행동은 다른 노드가 이 값을 보고 수행)
        # -----------------------------
        self.state_pub = self.create_publisher(
            String,
            '/mission/state',
            10,
        )

        self.target_type_pub = self.create_publisher(
            String,
            '/mission/target_type',
            10,
        )

        self.target_pub = self.create_publisher(
            PoseStamped,
            '/mission/current_target',
            10,
        )

        # 행동을 담당하는 노드가 현재 목적지를 다 처리했을 때 호출
        self.create_service(
            Trigger,
            '~/target_complete',
            self.target_complete_callback,
        )

        # State machine timer
        self.create_timer(
            self.get_parameter('state_check_period').value,
            self.timer_callback,
        )

    # =========================================================
    # Detections
    # =========================================================

    def detection_callback(self, msg):

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().warn(
                f'Invalid detection JSON: {e}'
            )
            return

        target_type = payload.get('class')

        if target_type not in ('fire', 'person'):
            return

        pose = Pose()
        pose.position.x = payload['x']
        pose.position.y = payload['y']
        pose.position.z = payload.get('z', 0.0)

        header = Header()
        header.frame_id = payload.get('frame_id', self.map_frame)

        self._add_detection(target_type, header, pose)

    def _add_detection(self, target_type, header, pose):

        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose = pose

        if self._find_found_target(target_type, pose) is not None:
            # 이미 기록된 대상 — pending/done 상관없이 재감지는
            # 위치 갱신도, 재방문 대상 추가도 하지 않고 무시한다.
            # 꺼졌는지 여부는 나중에 지도를 그릴 때 status 로만 확인한다.
            return

        entry = {
            'type': target_type,
            'pose': pose_stamped,
            'status': 'pending',
            # 근처의 반대 타입 대기 항목과 짝지어지면 그 dict 를
            # 가리킨다. 한 번 맺어지면 상대가 완료돼도 안 풀린다 —
            # '이 사람은 원래 저 불과 한 세트였다'는 사실 자체가
            # 우선순위 판단에 계속 쓰이기 때문.
            'partner': None,
        }

        partner_type = 'person' if target_type == 'fire' else 'fire'
        partner = self._find_unpaired_partner(partner_type, pose)

        if partner is not None:
            entry['partner'] = partner
            partner['partner'] = entry

        self.found_targets.append(entry)
        self.target_queue.append(entry)

    def _find_unpaired_partner(self, partner_type, pose):

        for entry in self.target_queue:

            if entry['type'] != partner_type or entry['partner'] is not None:
                continue

            if (
                self._planar_distance(entry['pose'].pose, pose)
                <= self.fire_person_proximity_threshold
            ):
                return entry

        return None

    def _find_found_target(self, target_type, pose):

        for entry in self.found_targets:

            if entry['type'] != target_type:
                continue

            if (
                self._planar_distance(entry['pose'].pose, pose)
                <= self.target_merge_radius
            ):
                return entry

        return None

    # =========================================================
    # Battery
    # =========================================================

    def battery_callback(self, msg):
        self.battery_raw = msg.data

    def is_battery_low(self):
        return (
            self.battery_raw is not None
            and self.battery_raw <= self.low_battery_threshold
        )

    # =========================================================
    # Frontier exploration target
    # =========================================================

    def frontier_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose
        self.frontier_target = pose_stamped

    # =========================================================
    # Target completion (다른 노드가 처리 완료를 알려줌)
    # =========================================================

    def target_complete_callback(self, request, response):

        if (
            self.active_target is not None
            and self.active_target in self.target_queue
        ):
            self.active_target['status'] = 'done'
            self.target_queue.remove(self.active_target)

        self.active_target = None

        response.success = True
        return response

    # =========================================================
    # Timer / State machine
    # =========================================================

    def timer_callback(self):
        self._update_robot_pose()
        self._refresh_state()

    def _refresh_state(self):

        if self.is_battery_low():
            self._enter_returning_to_base()
            return

        priority_target = self._pick_priority_target()

        if priority_target is not None:
            self._enter_urgent_target(priority_target)
            return

        self._enter_exploring()

    def _enter_returning_to_base(self):
        self.state = self.RETURNING_TO_BASE
        self.active_target = None
        self._publish('base', self.base_pose)

    def _enter_urgent_target(self, entry):
        self.state = (
            self.FIRE_DETECTED
            if entry['type'] == 'fire'
            else self.PERSON_DETECTED
        )
        self.active_target = entry

        # 불/사람이 새로 나타나면 원래 가려던 탐사 목적지는 버린다.
        # 큐가 다시 비었을 때 Frontier 로부터 새로 받은 값으로만 재개한다.
        self.frontier_target = None

        self._publish(entry['type'], entry['pose'])

    def _enter_exploring(self):
        self.state = self.EXPLORING
        self.active_target = None

        # state 만으로는 EXPLORING 안에서 "갈 곳이 정해졌는지(frontier)
        # 아직 없는지(idle)"를 구분할 수 없어서 target_type 으로 구분한다.
        # idle 일 땐 pose_stamped=None 이라 current_target 을 publish
        # 하지 않으므로, 구독자는 target_type=='frontier' 일 때만
        # current_target 을 유효한 값으로 보고 써야 한다 — 그렇지
        # 않으면 idle 중에도 이전 frontier 좌표가 그대로 남아있는
        # current_target 을 잘못 쓰게 된다.
        if self.frontier_target is not None:
            self._publish('frontier', self.frontier_target)
        else:
            self._publish('idle', None)

    def _publish(self, target_type, pose_stamped):

        if self.state != self._last_logged_state:
            self.get_logger().info(
                f'State -> {self.state} ({target_type})'
            )
            self._last_logged_state = self.state

        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        type_msg = String()
        type_msg.data = target_type
        self.target_type_pub.publish(type_msg)

        if pose_stamped is not None:
            self.target_pub.publish(pose_stamped)

    # =========================================================
    # Target priority (가까운 목적지부터 처리)
    # =========================================================

    def _pick_priority_target(self):
        """
        지금 상황에서 가장 먼저 처리해야 할 목적지를 고른다.

        이미 active_target 으로 접근 중인 불/사람이 있으면 그대로
        유지한다 — 유일한 예외는 지금 접근 중인 사람의 partner 로
        새로 불이 잡힌 경우(그 사람 바로 옆에 불이 막 생긴 것)뿐이고,
        이미 불을 끄러 가는 중이면 무슨 일이 있어도 안 바꾼다.
        """

        if (
            self.active_target is not None
            and self.active_target in self.target_queue
        ):
            if self.active_target['type'] == 'person':
                partner = self.active_target['partner']

                if partner is not None and partner['status'] == 'pending':
                    return partner

            return self.active_target

        return self._choose_new_target()

    def _choose_new_target(self):
        """
        active_target 이 없을 때(처음이거나 방금 완료됐을 때) 큐
        전체에서 새로 목적지를 고른다.

        불-사람 짝(partner)은 감지 시점에 한 번 맺어지면 상대가
        완료돼도 풀리지 않는다 — 그래서 "짝이 있고, 아직 처리할
        차례인" 항목을 최우선으로 본다:

          - 짝이 있는 불: 항상 우선 (사람이 위험하니 먼저 끈다)
          - 짝이 있는 사람: 그 짝 불이 이미 꺼졌을 때만 (짝 불이
            아직 안 꺼졌으면 그 불부터 처리할 차례라 후보에서 제외)

        위 후보가 하나도 없으면(짝 없는 고립된 불/사람만 남음),
        로봇과 가장 가까운 것을 고른다.
        """

        if not self.target_queue:
            return None

        combo_ready = [
            entry for entry in self.target_queue
            if self._is_combo_ready(entry)
        ]

        if combo_ready:
            return self._nearest_to_robot(combo_ready)

        isolated = [
            entry for entry in self.target_queue
            if entry['partner'] is None
        ]

        if isolated:
            return self._nearest_to_robot(isolated)

        return None

    def _is_combo_ready(self, entry):

        if entry['partner'] is None:
            return False

        if entry['type'] == 'fire':
            return True

        return entry['partner']['status'] == 'done'

    def _nearest_to_robot(self, entries):

        if self.robot_x is None or self.robot_y is None:
            # 로봇 위치를 아직 모르면 먼저 들어온 목적지를 그대로 사용
            return entries[0]

        return min(
            entries,
            key=lambda entry: self._planar_distance_xy(
                self.robot_x,
                self.robot_y,
                entry['pose'].pose,
            ),
        )

    # =========================================================
    # Helpers
    # =========================================================

    def _update_robot_pose(self):

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )

        except TransformException:
            return

        self.robot_x = transform.transform.translation.x
        self.robot_y = transform.transform.translation.y

    def _planar_distance(self, pose_a, pose_b):
        return math.hypot(
            pose_a.position.x - pose_b.position.x,
            pose_a.position.y - pose_b.position.y,
        )

    def _planar_distance_xy(self, x, y, pose):
        return math.hypot(
            pose.position.x - x,
            pose.position.y - y,
        )

    def _build_base_pose(self):

        base_x = self.get_parameter('base_x').value
        base_y = self.get_parameter('base_y').value
        base_yaw = self.get_parameter('base_yaw').value

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.map_frame

        pose_stamped.pose.position.x = base_x
        pose_stamped.pose.position.y = base_y

        pose_stamped.pose.orientation.z = math.sin(base_yaw / 2.0)
        pose_stamped.pose.orientation.w = math.cos(base_yaw / 2.0)

        return pose_stamped


def main(args=None):

    rclpy.init(args=args)

    node = StateManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
