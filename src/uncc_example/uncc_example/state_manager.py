import json
import math
from collections import deque

import rclpy

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener, TransformException

from std_msgs.msg import String, UInt16
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
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

        # -----------------------------
        # State
        # -----------------------------
        self.state = self.EXPLORING

        self.robot_x = None
        self.robot_y = None

        # 로봇이 처음 켜졌을 때(TF 가 처음 잡힌 순간)의 map 좌표.
        # 이후 어떤 target 을 거치든 값이 절대 안 바뀐다 — 이후 '복귀'
        # 는 이 좌표를 기준으로 한다.
        self.start_x = None
        self.start_y = None

        self.battery_raw = None
        # 배터리 raw 값이 임계치 근처에서 순간적으로 튀는 걸(전류 부하로
        # 인한 전압 강하 등) 걸러내기 위해 최근 10개의 평균으로 판단한다.
        self.battery_history = deque(maxlen=10)

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

        # 발견 기록 전체 (원본 데이터만, 시각화는 별도 노드
        # map_visualizer 가 담당). 같은 도메인의 서버가 구독해서
        # MarkerArray 로 바꿔 RViz2 에 그린다. found_targets 가
        # 바뀔 때마다(신규 발견/완료 처리) 전체 스냅샷을 다시 보낸다.
        self.found_targets_pub = self.create_publisher(
            String,
            '/mission/found_targets',
            10,
        )

        # 행동을 담당하는 노드가 현재 목적지를 다 처리했을 때 호출.
        # request.data 로 실제 성공 여부(불이 꺼졌는지 등)를 같이
        # 받는다 — 완료 처리와 결과값을 한 번에 원자적으로 처리하기
        # 위해 Trigger 대신 SetBool 사용 (별도 토픽으로 따로 받으면
        # active_target 이 이미 지워진 뒤일 수 있는 레이스가 생김).
        self.create_service(
            SetBool,
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

        frame_id = payload.get('frame_id', self.map_frame)
        new_entries = []

        # 한 프레임의 감지를 전부 먼저 추가만 해두고(무리 묶기는
        # 아직 안 함) — 도착 순서가 아니라 프레임 전체를 놓고 거리로
        # 무리를 정해야 하므로, 다 추가한 뒤 한 번에 처리한다.
        for detection in payload.get('detections', []):

            target_type = detection.get('class')

            if target_type not in ('fire', 'person'):
                continue

            pose_stamped = self._make_pose_stamped(
                frame_id,
                detection['x'],
                detection['y'],
            )

            entry = self._add_detection(target_type, pose_stamped)

            if entry is not None:
                new_entries.append(entry)

        if new_entries:
            self._update_clusters()
            self._publish_found_targets()

    def _add_detection(self, target_type, pose_stamped):
        """
        중복(이미 기록된 대상)이면 아무것도 안 하고 None 반환.
        새로 추가됐으면 그 entry 를 반환 — 무리 묶기는 여기서 안
        하고 detection_callback 이 프레임 전체를 다 추가한 뒤
        _update_clusters 에서 한꺼번에 처리한다.
        """

        pose = pose_stamped.pose

        if self._find_found_target(target_type, pose) is not None:
            # 이미 기록된 대상 — pending/done 상관없이 재감지는
            # 위치 갱신도, 재방문 대상 추가도 하지 않고 무시한다.
            # 꺼졌는지 여부는 나중에 지도를 그릴 때 status 로만 확인한다.
            return None

        entry = {
            'type': target_type,
            'pose': pose_stamped,
            'status': 'pending',
            # 임계값 이내로 묶인 무리(리스트, 무리원끼리 같은 리스트
            # 객체 공유). 대표 하나만 짝짓지 않고 무리 전체가 같은
            # 우선순위를 공유하도록 하기 위함.
            'cluster': None,
            # fire 만 의미 있음 — target_complete_callback 이
            # 실제 진화 성공 여부를 여기에 채운다 (지도 표시용).
            'extinguished': None,
        }

        self.found_targets.append(entry)
        self.target_queue.append(entry)

        return entry

    def _update_clusters(self):
        """
        아직 무리가 없는 불/사람 중, 임계값 이내인 불-사람 쌍을
        전부 하나의 무리로 묶는다(연쇄적으로도 합쳐짐).
        """

        unclustered = [
            entry for entry in self.target_queue
            if entry['cluster'] is None
        ]

        fires = [entry for entry in unclustered if entry['type'] == 'fire']
        persons = [
            entry for entry in unclustered if entry['type'] == 'person'
        ]

        for fire in fires:
            for person in persons:

                distance = self._planar_distance_xy(
                    fire['pose'].pose.position.x,
                    fire['pose'].pose.position.y,
                    person['pose'].pose,
                )

                if distance <= self.fire_person_proximity_threshold:
                    self._merge_into_cluster(fire, person)

    def _merge_into_cluster(self, a, b):

        cluster_a = a['cluster']
        cluster_b = b['cluster']

        if cluster_a is None and cluster_b is None:
            cluster = [a, b]
            a['cluster'] = cluster
            b['cluster'] = cluster
        elif cluster_a is None:
            cluster_b.append(a)
            a['cluster'] = cluster_b
        elif cluster_b is None:
            cluster_a.append(b)
            b['cluster'] = cluster_a
        elif cluster_a is not cluster_b:
            # 원래 다른 두 무리였는데 이번 연결로 하나가 됨
            cluster_a.extend(cluster_b)
            for entry in cluster_b:
                entry['cluster'] = cluster_a

    def _find_found_target(self, target_type, pose):

        for entry in self.found_targets:

            if entry['type'] != target_type:
                continue

            if (
                self._planar_distance_xy(
                    pose.position.x, pose.position.y, entry['pose'].pose
                )
                <= self.target_merge_radius
            ):
                return entry

        return None

    def _publish_found_targets(self):

        payload = {
            'targets': [
                {
                    'type': self._target_category(entry),
                    'x': entry['pose'].pose.position.x,
                    'y': entry['pose'].pose.position.y,
                }
                for entry in self.found_targets
            ],
        }

        msg = String()
        msg.data = json.dumps(payload)

        self.found_targets_pub.publish(msg)

    def _target_category(self, entry):
        """
        지도 표시용 5분류: person_unconfirmed / person_confirmed /
        fire_unvisited / fire_failed / fire_extinguished. 내부
        로직(우선순위 판단 등)에 쓰는 type/status 는 그대로 두고,
        publish 할 때만 하나로 합친다 — 구독하는 쪽(map_visualizer)이
        내부 상태 의미를 몰라도 되게 하기 위함.

        사람은 아직 구조 성공/실패를 알려주는 노드가 없어서(fire_
        extinguisher 같은 게 없음) 방문 여부(status)만으로 나뉜다.
        """

        if entry['type'] == 'person':
            if entry['status'] == 'done':
                return 'person_confirmed'
            return 'person_unconfirmed'

        if entry['status'] != 'done':
            return 'fire_unvisited'

        if entry['extinguished']:
            return 'fire_extinguished'

        return 'fire_failed'

    # =========================================================
    # Battery
    # =========================================================

    def battery_callback(self, msg):
        self.battery_raw = msg.data
        self.battery_history.append(msg.data)

    def is_battery_low(self):

        if len(self.battery_history) < 10:
            return False

        average = sum(self.battery_history) / len(self.battery_history)
        low = average <= self.low_battery_threshold

        if low:
            self.get_logger().warn(
                f'배터리 부족 (최근 {len(self.battery_history)}개 평균 '
                f'{average:.0f} <= 임계값 {self.low_battery_threshold})',
                throttle_duration_sec=5.0,
            )

        return low

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

        # active_target 이 있으면 항상 target_queue 안에도 있다 —
        # 여기서 빼는 게 유일한 제거 경로이자 active_target 을 None
        # 으로 되돌리는 지점이기도 하기 때문에 멤버십 재확인이 불필요.
        if self.active_target is not None:
            self.active_target['status'] = 'done'
            # fire 는 request.data 로 실제 진화 성공 여부가 들어온다
            # (person/base 는 의미 없지만 넣어도 무해함).
            self.active_target['extinguished'] = request.data
            self.target_queue.remove(self.active_target)
            self._publish_found_targets()

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

        start_pose = self._make_pose_stamped(
            self.map_frame, self.start_x, self.start_y
        )

        self._publish('base', start_pose)

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
        유지한다 — 유일한 예외는 지금 접근 중인 사람의 무리에 새로
        불이 잡힌 경우(그 사람 근처에 불이 막 생긴 것)뿐이고,
        이미 불을 끄러 가는 중이면 무슨 일이 있어도 안 바꾼다.
        """

        if self.active_target is not None:
            if self.active_target['type'] == 'person':
                cluster = self.active_target['cluster'] or []
                pending_fires = [
                    entry for entry in cluster
                    if entry['type'] == 'fire'
                    and entry['status'] == 'pending'
                ]

                if pending_fires:
                    return self._nearest_to_robot(pending_fires)

            return self.active_target

        return self._choose_new_target()

    def _choose_new_target(self):
        """
        active_target 이 없을 때(처음이거나 방금 완료됐을 때) 큐
        전체에서 새로 목적지를 고른다.

        무리(cluster)는 감지 시점에 한 번 묶이면 무리원이 완료돼도
        풀리지 않는다 — 그래서 "무리가 있고, 아직 처리할 차례인"
        항목을 최우선으로 본다:

          - 무리가 있는 불: 항상 우선 (사람이 위험하니 먼저 끈다)
          - 무리가 있는 사람: 그 무리의 불이 전부 이미 꺼졌을
            때만 (안 꺼진 불이 하나라도 있으면 그 불부터 처리할
            차례라 후보에서 제외)

        위 후보가 하나도 없으면(무리 없는 고립된 불/사람만 남음),
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
            if entry['cluster'] is None
        ]

        if isolated:
            return self._nearest_to_robot(isolated)

        return None

    def _is_combo_ready(self, entry):

        if entry['cluster'] is None:
            return False

        if entry['type'] == 'fire':
            return True

        return all(
            member['status'] == 'done'
            for member in entry['cluster']
            if member['type'] == 'fire'
        )

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

        if self.start_x is None and self.start_y is None:
            # TF 가 처음 잡힌 첫 tick 에서 딱 한 번만 저장한다.
            self.start_x = self.robot_x
            self.start_y = self.robot_y

    def _planar_distance_xy(self, x, y, pose):
        return math.hypot(
            pose.position.x - x,
            pose.position.y - y,
        )

    def _make_pose_stamped(self, frame_id, x, y):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame_id
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
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
