import json
import math
import time

import rclpy

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener, TransformException

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String, UInt16
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from interfaces.srv import SetString

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
<<<<<<< HEAD
        # 튄 좌표가 그대로 target 이 되는 걸 막기 위함(연속일 필요는 없음)
        self.declare_parameter('target_confirm_hits', 2)
=======
        # 튄 좌표가 그대로 target 이 되는 걸 막기 위함(연속일 필요는 없음).
        # person은 화면에 짧게 보이는 경우가 많아 fire보다 낮게 둔다.
        self.declare_parameter('fire_target_confirm_hits', 3)
        self.declare_parameter('person_target_confirm_hits', 2)
>>>>>>> 879912b (feat: 객체마다 다른 누적 검출 횟수 적용)

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
            f"({confirmed_position.x:.2f}, {confirmed_position.y:.2f})"
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
            self.get_logger().warn(
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

        response.success = True
        return response

    # =========================================================
    # Mission start signal (bringup 완료 후 운용자가 호출)
    # =========================================================

    def start_mission_callback(self, request, response):

        if self._mission_started:
            response.success = True
            response.message = 'Mission already started'
            return response

        self._mission_started = True
        self._event_logger.info('Mission start signal received')

        response.success = True
        response.message = 'Mission started'
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

        if self.is_battery_low():
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

        if pose_stamped is None:
            return

        target_id = id(self.active_target)

        if state_changed or target_id != self._last_published_target_id:
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
