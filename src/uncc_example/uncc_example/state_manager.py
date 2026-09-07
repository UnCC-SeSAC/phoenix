import json
import math
import time

import rclpy

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener, TransformException

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Bool, String, UInt16
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from interfaces.srv import SetString

# slam_toolbox 전용 커스텀 서비스(slam_toolbox.srv.Reset)에 의존하지 않고,
# 모든 rclcpp_lifecycle 노드가 표준으로 제공하는 lifecycle 프로토콜만으로
# 맵을 리셋한다 — sync_slam_toolbox_node 는 LifecycleNode 라 deactivate ->
# cleanup -> configure -> activate 를 거치면 내부 pose graph/mapper 가
# 전부 새로 초기화된다(노드 재시작과 동일한 효과, 프로세스는 안 죽음).
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState

from .log_utils import make_event_logger


class StateManager(Node):

    # bringup 직후 초기 상태: start_mission 서비스로 신호를 받기 전까지
    # 대기하며, frontier/nav2 쪽으로 어떤 START 요청도 보내지 않는다
    STANDBY = 'STANDBY'
    # 화재/인명 정보가 전혀 없을 때: Frontier 가 알려주는 미탐사 위치로 이동
    EXPLORING = 'EXPLORING'
    # 대기중인 목적지 중 사람이 가장 가까움: 구조 접근
    PERSON_DETECTED = 'PERSON_DETECTED'
    # 대기중인 목적지 중 불이 가장 가까움: 진화 접근
    FIRE_DETECTED = 'FIRE_DETECTED'
    # 배터리가 임계치 이하: 다른 목적지보다 우선하여 충전하러 복귀
    # (frontier_exploration_ros2 의 return_to_start 와는 다른 개념이라 구분)
    RETURNING_TO_CHARGE = 'RETURNING_TO_CHARGE'
    RETURNING_TO_BASE = 'RETURNING_TO_BASE'
    RETURNING_MANUAL = 'RETURNING_MANUAL'

    # target_complete 요청(request.data)에 담기는 처리 결과 문자열.
    # mission_executor 가 그대로 가져다 쓰므로 여기서만 정의한다.
    TARGET_STATUS_SUCCESS = 'success'          # person 도달 / fire 진화 성공
    TARGET_STATUS_FAILED = 'failed'            # fire 진압을 시도했으나 실패
    TARGET_STATUS_UNREACHABLE = 'unreachable'  # nav2 반복 실패로 시도 자체를 못함

    def __init__(self):
        super().__init__('state_manager')

        self._event_logger = make_event_logger(self)

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')

        # 로봇 컨트롤러가 보내는 raw 배터리 값 기준 (기기에 맞게 튜닝 필요)
        self.declare_parameter('low_battery_threshold', 7000)

        # raw 값이 임계값 기준 방향을 이만큼(초) 연속 유지해야 상태를 전환한다
        self.declare_parameter('low_battery_confirm_sec', 3.0)

        # 이 반경 안의 같은 종류(fire/person) 감지는 하나의 목적지로 병합
        self.declare_parameter('target_merge_radius', 0.5)

        # 같은 위치(target_merge_radius 이내)에서 이 횟수만큼 감지돼야
        # 확정된 target 으로 등록한다 — depth/TF 지연으로 한두 프레임
        # 튄 좌표가 그대로 target 이 되는 걸 막기 위함(연속일 필요는 없음).
        # person은 화면에 짧게 보이는 경우가 많아 fire보다 낮게 둔다.
        self.declare_parameter('fire_target_confirm_hits', 3)
        self.declare_parameter('person_target_confirm_hits', 2)

        # 불-사람이 이 거리 이내로 붙어 있으면 사람이 위험하다고 보고
        # 불부터 끄고, 멀면 사람부터 확인한다
        self.declare_parameter('fire_person_proximity_threshold', 0.3)

        self.declare_parameter('state_check_period', 0.2)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.low_battery_threshold = (
            self.get_parameter('low_battery_threshold').value
        )
        self.low_battery_confirm_sec = (
            self.get_parameter('low_battery_confirm_sec').value
        )
        self.target_merge_radius = (
            self.get_parameter('target_merge_radius').value
        )
        self.fire_target_confirm_hits = (
            self.get_parameter('fire_target_confirm_hits').value
        )
        self.person_target_confirm_hits = (
            self.get_parameter('person_target_confirm_hits').value
        )
        self.fire_person_proximity_threshold = (
            self.get_parameter('fire_person_proximity_threshold').value
        )

        # -----------------------------
        # State
        # -----------------------------
        self.state = self.STANDBY
        # start_mission 서비스가 호출되기 전까지 True 로 안 바뀐다
        self._mission_started = False
        # stop_mission 서비스로 켜짐 — 배터리와 무관하게 강제로 복귀시키고,
        # start_mission 이 다시 호출되기 전까지 유지된다
        self._manual_stop = False

        self.robot_x = None
        self.robot_y = None

        # TF 가 처음 잡힌 순간의 map 좌표 — 이후 안 바뀌며, '복귀'의 기준점.
        self.start_x = None
        self.start_y = None

        # 임계값 기준 방향이 low_battery_confirm_sec 만큼 유지돼야 확정한다
        self.latest_battery = None
        self._battery_low_state = False
        self._battery_pending_value = None
        self._battery_pending_since = None

        # fire/person 목적지 큐. found_targets 와 원소(dict)를 공유하며,
        # 아직 처리 안 한 것만 담고 target_complete 로 여기서만 빠진다.
        self.target_queue = []
        self.active_target = None  # 현재 상태로 채택된 목적지 (완료 통보 대상)

        # 미션 시작부터 끝까지 유지되는 발견 기록 (완료된 것도 안 지움).
        # 신규 감지 중복 판단(dedup)과, 나중에 지도에 표시할 데이터로 쓴다.
        self.found_targets = []

        # fire/person별 confirm hits 횟수만큼 감지되기 전까지 대기하는 후보 목록.
        # {'type', 'pose'(최초 감지 좌표, 갱신 안 함), 'hits'} 원소.
        self._pending_candidates = []

        self._last_published_state = None
        self._last_published_target_id = None
        self._last_published_manual_stop = None

        # 카메라는 SLAM 이 아직 못 그린 영역도 멀리서 감지할 수 있어서,
        # 감지된 좌표가 지금 global costmap 범위 밖일 수 있다 — 그 상태로
        # active_target 으로 넘기면 nav2 가 "off the global costmap"으로
        # 매번 실패한다. 범위 밖 후보는 여기서 걸러서 EXPLORING 이 계속
        # 그쪽을 탐사하다 맵이 커지면 그때 선택되게 한다.
        self._map_info = None  # (resolution, width, height, origin_x, origin_y)

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

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self._map_callback,
            map_qos,
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
        # Publishers (실제 행동은 다른 노드가 이 값을 보고 수행)
        # -----------------------------
        self.state_pub = self.create_publisher(
            String,
            '/mission/state',
            10,
        )

        # UI 표시 전용 신호 — RETURNING_TO_CHARGE 가 배터리 때문인지
        # stop_mission(수동 정지) 때문인지 구분하려고 별도로 알린다.
        # /mission/state 자체는 mission_executor 가 그대로 비교하는
        # 값이라 바꾸면 안 된다.
        self.manual_stop_pub = self.create_publisher(
            Bool,
            '/mission/manual_stop',
            10,
        )

        self.target_pub = self.create_publisher(
            PoseStamped,
            '/mission/current_target',
            10,
        )

        # 발견 기록 전체(원본 데이터, 시각화는 map_visualizer 가 담당).
        # found_targets 가 바뀔 때마다 전체 스냅샷을 다시 보낸다.
        self.found_targets_pub = self.create_publisher(
            String,
            '/mission/found_targets',
            10,
        )

        # stop_mission 으로 홈에 복귀한 뒤, 다음 탐사가 예전 맵(예: 그 사이
        # 열렸다 닫힌 문처럼 지금은 안 맞는 정보)이 아니라 지금 라이다가
        # 보는 그대로에서 다시 시작하도록 slam_toolbox 를 lifecycle
        # 전환으로 리셋한다 (change_state 는 모든 LifecycleNode가 표준으로
        # 제공하므로 slam_toolbox 버전/설치본에 따라 달라지지 않는다).
        self._slam_lifecycle_client = self.create_client(
            ChangeState,
            '/slam_toolbox/change_state',
        )

        # 행동 노드가 목적지 처리를 끝내면 호출. request.data 에 처리 결과
        # (TARGET_STATUS_*)를 담아 보낸다 — bool 하나로는 "실패"와
        # "시도조차 못함(unreachable)"을 구분 못해 SetBool 대신 SetString 사용.
        self.create_service(
            SetString,
            '~/target_complete',
            self.target_complete_callback,
        )

        # bringup(카메라/YOLO/서보모터 등)이 전부 끝난 뒤 운용자가 직접
        # 호출해서 탐사를 시작시키는 신호.
        # ros2 service call /state_manager/start_mission std_srvs/srv/Trigger "{}"
        self.create_service(
            Trigger,
            '~/start_mission',
            self.start_mission_callback,
        )

        # UI의 STOP 버튼 등에서 호출 — 배터리와 무관하게 강제로
        # RETURNING_TO_CHARGE 로 보내고, start_mission 이 다시 호출되기
        # 전까지 그 상태를 유지한다.
        # ros2 service call /state_manager/stop_mission std_srvs/srv/Trigger "{}"
        self.create_service(
            Trigger,
            '~/stop_mission',
            self.stop_mission_callback,
        )

        # State machine timer
        self.create_timer(
            self.get_parameter('state_check_period').value,
            self.timer_callback,
        )

    # =========================================================
    # /map — global costmap 범위 밖 목적지를 걸러내기 위한 기준.
    # =========================================================

    def _map_callback(self, msg):

        self._map_info = {
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
        }

    def _is_within_map(self, entry):
        """맵을 아직 못 받았으면(초기 구동 중) 보수적으로 범위 밖 취급 —
        확실해질 때까지 active_target 으로 넘기지 않는다."""

        if self._map_info is None:
            return False

        info = self._map_info
        position = entry['pose'].pose.position

        min_x = info['origin_x']
        min_y = info['origin_y']
        max_x = min_x + info['width'] * info['resolution']
        max_y = min_y + info['height'] * info['resolution']

        return min_x <= position.x <= max_x and min_y <= position.y <= max_y

    # =========================================================
    # Detections
    # =========================================================

    def detection_callback(self, msg):

        if not self._mission_started:
            # STANDBY 동안은 vision 파이프라인이 계속 돌더라도 감지 결과를
            # found_targets/target_queue 에 쌓지 않는다 — start_mission
            # 즉시 이전 감지로 EXPLORING 을 건너뛰고 바로 타겟으로 직행하는
            # 걸 막기 위함.
            return

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
        """이미 확정된 대상과 겹치면 None. 아직 확정 전(후보)이면 카운트만
        올리고 객체 종류별 confirm hits 미달이면 None. 도달하면 그때 비로소
        확정해 등록하고 그 entry 를 반환한다. 무리 묶기는 여기서 안 하고
        _update_clusters 에서 한꺼번에 처리한다."""

        pose = pose_stamped.pose

        if self._find_found_target(target_type, pose) is not None:
            # 이미 확정된 대상은 다시 감지돼도 무시한다(위치 갱신 안 함).
            return None

        candidate = self._find_pending_candidate(target_type, pose)

        if candidate is None:
            self._pending_candidates.append({
                'type': target_type,
                'pose': pose_stamped,
                'hits': 1,
            })
            return None

        candidate['hits'] += 1

        required_hits = (
            self.fire_target_confirm_hits
            if target_type == 'fire'
            else self.person_target_confirm_hits
        )

        if candidate['hits'] < required_hits:
            return None

        self._pending_candidates.remove(candidate)

        # 확정 좌표는 후보의 최초 감지 좌표를 그대로 쓴다(갱신 안 함).
        confirmed_pose = candidate['pose']

        # position은 실제 객체 좌표로 유지하고, 객체를 확정했을 때의
        # 로봇→객체 시선 방향을 orientation에 저장한다. 나중에 다른 객체
        # 위치에서 접근 방향을 다시 만들지 않기 위한 힌트다.
        heading_saved = False
        if (
            confirmed_pose.header.frame_id == self.map_frame
            and self.robot_x is not None
            and self.robot_y is not None
        ):
            position = confirmed_pose.pose.position
            heading = math.atan2(
                position.y - self.robot_y,
                position.x - self.robot_x,
            )
            confirmed_pose.pose.orientation.z = math.sin(heading / 2.0)
            confirmed_pose.pose.orientation.w = math.cos(heading / 2.0)
            heading_saved = True
        else:
            # Identity quaternion을 저장 방향 0rad로 오해하지 않게 명시적인
            # '힌트 없음' 값으로 둔다. mission_executor는 norm으로 구분한다.
            confirmed_pose.pose.orientation.z = 0.0
            confirmed_pose.pose.orientation.w = 0.0

        entry = {
            'type': target_type,
            'pose': confirmed_pose,
            'status': 'pending',
            # 임계값 이내로 묶인 무리. 무리원끼리 같은 리스트를 공유해서
            # 무리 전체가 같은 우선순위를 갖게 한다.
            'cluster': None,
            # fire 전용 — target_complete_callback 이 진화 성공 여부를 채운다.
            'extinguished': None,
            # nav2 가 반복 실패해서 실제 임무(구조/진압)를 시도조차 못했는지.
            'unreachable': False,
        }

        self.found_targets.append(entry)
        self.target_queue.append(entry)

        confirmed_position = confirmed_pose.pose.position

        self._event_logger.info(
            f"새 객체 인식: {target_type} "
            f"({confirmed_position.x:.2f}, {confirmed_position.y:.2f}) "
            f"approach_heading_saved={heading_saved}"
        )

        return entry

    def _find_pending_candidate(self, target_type, pose):

        for candidate in self._pending_candidates:

            if candidate['type'] != target_type:
                continue

            if (
                self._planar_distance_xy(
                    pose.position.x, pose.position.y, candidate['pose'].pose
                )
                <= self.target_merge_radius
            ):
                return candidate

        return None

    def _update_clusters(self):
        """무리 없는 불/사람 중 임계값 이내인 불-사람 쌍을 무리로 묶는다
        (여러 쌍이 걸리면 연쇄적으로 하나의 무리로 합쳐진다)."""

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
        """지도 표시용 7분류로 변환한다: person_unconfirmed / person_confirmed
        / person_unreachable / fire_unvisited / fire_failed /
        fire_extinguished / fire_unreachable. 내부 로직에 쓰는 type/status 는
        그대로 두고 publish 할 때만 하나로 합쳐서, 구독하는 쪽(map_visualizer)
        이 내부 상태를 몰라도 되게 한다.

        *_unreachable 은 nav2 가 반복 실패해서 구조/진압을 시도조차 못한
        경우다. 사람은 구조 성공/실패를 알려주는 노드가 없어서 나머지는
        방문 여부(status)로만 구분한다."""

        if entry['type'] == 'person':
            if entry['status'] != 'done':
                return 'person_unconfirmed'
            if entry['unreachable']:
                return 'person_unreachable'
            return 'person_confirmed'

        if entry['status'] != 'done':
            return 'fire_unvisited'

        if entry['unreachable']:
            return 'fire_unreachable'

        if entry['extinguished']:
            return 'fire_extinguished'

        return 'fire_failed'

    # =========================================================
    # Battery
    # =========================================================

    def battery_callback(self, msg):
        self.latest_battery = msg.data

    def is_battery_low(self):

        if self.latest_battery is None:
            return False

        raw_low = self.latest_battery <= self.low_battery_threshold
        now = time.time()

        if raw_low != self._battery_pending_value:
            self._battery_pending_value = raw_low
            self._battery_pending_since = now

        if (
            raw_low != self._battery_low_state
            and self._battery_pending_since is not None
            and now - self._battery_pending_since
            >= self.low_battery_confirm_sec
        ):
            self._battery_low_state = raw_low

        if self._battery_low_state:
            self._event_logger.warn(
                f'배터리 부족 (raw {self.latest_battery} <= '
                f'임계값 {self.low_battery_threshold})',
                throttle_duration_sec=5.0,
            )

        return self._battery_low_state

    # =========================================================
    # Target completion (다른 노드가 처리 완료를 알려줌)
    # =========================================================

    def target_complete_callback(self, request, response):

        # active_target 은 항상 target_queue 안에 있다 — 여기가 유일한
        # 제거 경로라 따로 존재 확인을 하지 않아도 된다.
        if self.active_target is not None:
            self.active_target['status'] = 'done'
            self.active_target['unreachable'] = (
                request.data == self.TARGET_STATUS_UNREACHABLE
            )
            # fire 는 request.data 로 실제 진화 성공 여부가 들어온다
            # (person/base 는 의미 없지만 넣어도 무해함).
            self.active_target['extinguished'] = (
                request.data == self.TARGET_STATUS_SUCCESS
            )
            self.target_queue.remove(self.active_target)
            self._publish_found_targets()

        self.active_target = None

        # stop_mission 으로 복귀 중이었다면 이 호출이 곧 "홈 도착" 신호다
        # (RETURNING_TO_CHARGE 는 active_target 이 없어서 위 분기를 안 타지만
        # nav2 도착 시 mission_executor 가 그래도 여기를 호출한다) — 이 기회에
        # 다음 start_mission 이 처음 탐사처럼 시작하도록 미션 기록을 지운다.
        if self.state == self.RETURNING_TO_CHARGE and self._manual_stop:
            self._reset_after_manual_stop()

        response.success = True
        return response

    def _reset_mission_records(self):
        self.target_queue = []
        self.found_targets = []
        self._pending_candidates = []
        self._publish_found_targets()
        self._call_slam_reset()

    def _reset_after_manual_stop(self):
        self._event_logger.info(
            'stop_mission: 홈 도착, 미션 기록을 지우고 STANDBY로 전환'
        )
        self._mission_started = False
        self._manual_stop = False
        self._reset_mission_records()

    # deactivate -> cleanup -> configure -> activate 순서로 반드시 이
    # 순서대로 밟아야 한다(cleanup 은 inactive 상태에서만 허용되는 등,
    # lifecycle 상태머신이 순서를 강제한다).
    _SLAM_RESET_SEQUENCE = (
        Transition.TRANSITION_DEACTIVATE,
        Transition.TRANSITION_CLEANUP,
        Transition.TRANSITION_CONFIGURE,
        Transition.TRANSITION_ACTIVATE,
    )

    def _call_slam_reset(self):
        if not self._slam_lifecycle_client.service_is_ready():
            self.get_logger().warning(
                '/slam_toolbox/change_state 서비스가 아직 준비되지 않음 — '
                '맵 리셋 건너뜀'
            )
            return

        self._send_slam_transition(0)

    def _send_slam_transition(self, step):
        if step >= len(self._SLAM_RESET_SEQUENCE):
            # map 원점이 리셋 시점 로봇 위치 근처로 다시 잡히므로, 예전
            # 좌표계 기준이던 start_x/y 를 버리고 다음 TF 갱신에서 새
            # map 좌표계 기준으로 다시 잡는다.
            self.start_x = None
            self.start_y = None
            self._event_logger.info(
                'slam_toolbox 맵 리셋 완료(lifecycle 재순환) — 시작 위치 재획득 대기'
            )
            return

        transition_id = self._SLAM_RESET_SEQUENCE[step]
        request = ChangeState.Request(transition=Transition(id=transition_id))

        def _on_response(future):
            try:
                response = future.result()
            except Exception as exc:  # noqa: BLE001 - 실패는 로깅만 함
                self.get_logger().error(
                    f'slam_toolbox lifecycle 전환(id={transition_id}) 호출 실패: '
                    f'{exc} — slam_toolbox 가 중간 상태에 멈춰 있을 수 있으니 '
                    "'ros2 lifecycle set /slam_toolbox activate' 로 수동 복구 필요"
                )
                return

            if not response.success:
                self.get_logger().error(
                    f'slam_toolbox lifecycle 전환(id={transition_id}) 거부됨 — '
                    "'ros2 lifecycle set /slam_toolbox activate' 로 수동 복구 필요"
                )
                return

            self._send_slam_transition(step + 1)

        self._slam_lifecycle_client.call_async(
            request
        ).add_done_callback(_on_response)

    # =========================================================
    # Mission start signal (bringup 완료 후 운용자가 호출)
    # =========================================================

    def start_mission_callback(self, request, response):

        was_started = self._mission_started
        self._manual_stop = False

        if was_started:
            # 처음 시작/(stop 후 홈 도착 완료) 둘 다 아니면 — 즉 미션이
            # 이미 도는 도중(홈 도착 전 stop 포함)이면 START는 재개가
            # 아니라 리셋 버튼으로 동작한다: 지금 상황 기준으로 초기화만
            # 하고 멈춰서, 실제로 움직이려면 STANDBY에서 START를 한 번
            # 더 눌러야 한다 (리셋 즉시 움직이지 않는다).
            self._reset_mission_records()
            self._mission_started = False
            self._event_logger.info(
                'Mission reset signal received — 대기 상태로 초기화'
            )
            response.success = True
            response.message = 'Mission reset'
            return response

        self._mission_started = True
        self._event_logger.info('Mission start signal received')

        response.success = True
        response.message = 'Mission started'
        return response

    def stop_mission_callback(self, request, response):

        if not self._mission_started:
            response.success = True
            response.message = 'Mission not started'
            return response

        self._manual_stop = True
        self._event_logger.info('Mission stop signal received (manual return to base)')

        response.success = True
        response.message = 'Returning to base'
        return response

    # =========================================================
    # Timer / State machine
    # =========================================================

    def timer_callback(self):
        self._update_robot_pose()
        self._refresh_state()

    def _refresh_state(self):

        if not self._mission_started:
            self._enter_standby()
            return

        if self._manual_stop or self.is_battery_low():
            self._enter_returning_to_charge()
            return

        priority_target = self._pick_priority_target()

        if priority_target is not None:
            self._enter_urgent_target(priority_target)
            return

        self._enter_exploring()

    def _enter_standby(self):
        self.state = self.STANDBY
        self.active_target = None

        self._publish(None)

    def _enter_returning_to_charge(self):
        self.state = self.RETURNING_TO_CHARGE
        self.active_target = None

        start_pose = self._make_pose_stamped(
            self.map_frame, self.start_x, self.start_y
        )

        self._publish(start_pose)

    def _enter_urgent_target(self, entry):
        self.state = (
            self.FIRE_DETECTED
            if entry['type'] == 'fire'
            else self.PERSON_DETECTED
        )
        self.active_target = entry

        self._publish(entry['pose'])

    def _enter_exploring(self):
        self.state = self.EXPLORING
        self.active_target = None

        self._publish(None)

    def _publish(self, pose_stamped):

        state_changed = self.state != self._last_published_state

        if state_changed:
            self._event_logger.info(f'State -> {self.state}')

            state_msg = String()
            state_msg.data = self.state
            self.state_pub.publish(state_msg)

            self._last_published_state = self.state

        if self._manual_stop != self._last_published_manual_stop:
            manual_stop_msg = Bool()
            manual_stop_msg.data = self._manual_stop
            self.manual_stop_pub.publish(manual_stop_msg)
            self._last_published_manual_stop = self._manual_stop

        if pose_stamped is None:
            return

        target_id = id(self.active_target)

        # Refresh the pose even if state/target topic delivery order differs.
        # MissionExecutor deduplicates navigation requests by state and target.
        self.target_pub.publish(pose_stamped)
        self._last_published_target_id = target_id

    # =========================================================
    # Target priority (가까운 목적지부터 처리)
    # =========================================================

    def _pick_priority_target(self):
        """지금 처리할 목적지를 고른다. active_target 이 있으면 그대로
        유지하되, 접근 중인 사람의 무리에 새 불이 잡힌 경우에만 그
        불로 전환한다 (이미 불을 끄러 가는 중이면 절대 안 바꾼다)."""

        if self.active_target is not None:
            if self.active_target['type'] == 'person':
                cluster = self.active_target['cluster'] or []
                pending_fires = [
                    entry for entry in cluster
                    if entry['type'] == 'fire'
                    and entry['status'] == 'pending'
                    and self._is_within_map(entry)
                ]

                if pending_fires:
                    return self._nearest_to_robot(pending_fires)

            return self.active_target

        return self._choose_new_target()

    def _choose_new_target(self):
        """active_target 이 없을 때 큐 전체에서 새 목적지를 고른다.

        무리(cluster)는 한 번 묶이면 무리원이 완료돼도 안 풀리므로,
        "무리가 있고 아직 처리할 차례인" 항목을 최우선으로 본다:
          - 무리가 있는 불: 항상 우선 (사람이 위험하니 먼저 끈다)
          - 무리가 있는 사람: 그 무리의 불이 전부 꺼졌을 때만

        해당 후보가 없으면(무리 없는 고립된 불/사람만 남으면) 로봇과
        가장 가까운 것을 고른다.

        어느 단계든 지금 global costmap 범위 밖인 후보는 건너뛴다 —
        카메라가 SLAM 이 아직 못 그린 곳도 감지할 수 있어서, 그런
        좌표를 바로 목적지로 넘기면 nav2 planner 가 매번 실패한다.
        범위 밖 후보는 그냥 두면 EXPLORING 이 계속 진행되다 맵이
        커지는 대로 다음 틱에 자연히 선택된다.
        """

        if not self.target_queue:
            return None

        combo_ready = [
            entry for entry in self.target_queue
            if self._is_combo_ready(entry) and self._is_within_map(entry)
        ]

        if combo_ready:
            return self._nearest_to_robot(combo_ready)

        isolated = [
            entry for entry in self.target_queue
            if entry['cluster'] is None and self._is_within_map(entry)
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
