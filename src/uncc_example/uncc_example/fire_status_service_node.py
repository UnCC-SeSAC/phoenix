#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fire_status_service_node

image_pipeline 의 yolo_node(YOLO26)가 발행하는 /yolo_result
(타입: vision_msgs/Detection2DArray)를 구독해서 최근 감지 이력을 쌓아두고,
CheckFireStatus 서비스 요청이 오면 그 이력을 집계해서 꺼졌는지 판별해 응답한다.

    /image_enhanced -> [yolo_node] -> /yolo_result -> [이 노드] -> check_fire_status
                                       Detection2DArray           (interfaces/srv)

image_pipeline 코드는 전혀 건드리지 않는다 - 이 노드가 결과만 구독한다.

--- 읽고 시작할 것 ---

★ /fire/detections 를 구독하면 안 된다.
  그건 메인에 나가는 **이벤트** 토픽이라 검출 0개면 아무것도 발행하지 않는다
  (detection_3d_node 의 publish_empty 기본 False). 그걸 구독하면
  "불이 꺼졌다"와 "노드가 죽었다"가 똑같이 침묵으로 보여서, 진압에
  성공해도 영원히 '안꺼짐'으로 응답하게 된다.
  /yolo_result 는 publish_empty 기본 True 라 검출 0개여도 빈 배열이
  매 프레임 온다. **그 빈 배열이 "불 없음"의 유일한 증거다.**

★ QoS 는 qos_profile_sensor_data 여야 한다.
  정수 10을 쓰면 RELIABLE 이 되어 yolo_node 의 BEST_EFFORT 발행과 매칭에
  실패한다. 에러는 안 나고 콜백만 영영 안 불린다.

★ 관찰 구간은 **과거**를 본다 (now - observation_seconds ~ now).
  그래서 호출 시점이 중요하다. 분사 직후에 바로 부르면 그 구간이
  **분사 중 프레임을 덮어서** 불이 방금 꺼졌어도 '안꺼짐'으로 기운다.
  fire_suppression_node 는 분사가 끝나고 observation_seconds 만큼
  기다린 뒤에 부른다 (STATUS_SETTLE_SECONDS).

★ FIRE_CLASS_NAME 은 YOLO 학습에 쓴 class_name 문자열과 정확히 같아야 한다.
  현재 데이터셋(datasets/fire/data.yaml)은 names: ['fire', 'person'] 이라
  'fire' 가 맞다. 대소문자만 달라도(예: 'Fire') 검출은 계속 오는데 화재만
  0건이 되어, 불이 타는 중에도 '꺼짐'으로 응답한다. 그래서 대소문자만
  다른 라벨이 오면 아래에서 한 번 크게 경고한다.

★ 로그는 log_utils 규약을 따른다 (기본 로거 ERROR, 이벤트 전용 child 로거).
  단 이 노드의 경고들은 "조용히 틀리는" 상황을 알리는 유일한 수단이라
  전부 event 로거로 내보낸다 - 기본 로거로 두면 ERROR 미만이 잘려 사라진다.
"""

from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from vision_msgs.msg import Detection2DArray

from interfaces.srv import CheckFireStatus

# vision_msgs 3.x/4.x 의 필드 모양 차이를 흡수한다 (result.hypothesis.class_id vs result.id).
from image_pipeline.detection_msgs import hypothesis

from .log_utils import make_event_logger


def _sensor_qos(depth: int) -> QoSProfile:
    """qos_profile_sensor_data 에서 큐 깊이만 바꾼다. reliability 는 그대로."""
    qos = QoSProfile(depth=int(depth))
    qos.reliability = qos_profile_sensor_data.reliability
    qos.durability = qos_profile_sensor_data.durability
    qos.history = qos_profile_sensor_data.history
    return qos


class FireStatusServiceNode(Node):
    def __init__(self):
        super().__init__('fire_status_service_node')

        # 기본 로거는 ERROR 로 낮추고, 동작과 직결된 로그만 이 child 로거로 낸다.
        self._event_logger = make_event_logger(self)

        # ---------------- 파라미터 ----------------
        # 코드를 고치지 않고 런치에서 조정할 수 있어야 한다. 특히
        # extinguished_ratio 와 min_score 는 실측으로 정할 값이다.
        self.declare_parameter('detections_topic', '/yolo_result')
        self.declare_parameter('service_name', 'check_fire_status')
        self.declare_parameter('fire_class_name', 'fire')
        # 이 점수 미만 검출은 없는 것으로 본다. 오검출 1건이 '안꺼짐'을
        # 붙드는 걸 막는다. 0.0 = 끔.
        # ★ 실제 /yolo_result 의 score 분포를 보고 정할 것. 아래 통계 로그가
        #   주기적으로 min/평균/max 를 찍어준다.
        self.declare_parameter('min_score', 0.0)
        # 관찰 구간 내 화재 프레임 비율이 이 값 **미만**이면 '꺼짐'.
        # ★ 15Hz면 3초에 45프레임이 쌓인다. 실측 재조정 대상.
        self.declare_parameter('extinguished_ratio', 0.3)
        # 요청 가능한 최대 관찰 구간. 이보다 큰 요청은 잘라내고 경고한다.
        self.declare_parameter('max_observation_sec', 5.0)
        # 히스토리 창. ★ max_observation_sec 보다 반드시 커야 한다.
        #   같게 두면 가장 오래된 표본이 경계에서 잘려나가 요청한 구간을
        #   온전히 못 본다. 최대 5초 요청에 6초 창으로 1초 여유를 둔다.
        self.declare_parameter('history_window_sec', 6.0)
        # 요청이 0 이하로 오면 쓸 기본값.
        self.declare_parameter('default_observation_sec', 3.0)
        self.declare_parameter('qos_depth', 30)
        # 0 이하면 주기 통계 로그를 끈다 (튜닝이 끝난 뒤 조용하게 돌리기 위함).
        self.declare_parameter('stats_period_sec', 5.0)

        p = self.get_parameter
        self._topic = str(p('detections_topic').value)
        self._fire_class = str(p('fire_class_name').value)
        self._min_score = float(p('min_score').value)
        self._ratio_threshold = float(p('extinguished_ratio').value)
        self._max_obs = float(p('max_observation_sec').value)
        self._window = float(p('history_window_sec').value)
        self._default_obs = float(p('default_observation_sec').value)

        if self._window <= self._max_obs:
            self._event_logger.warn(
                f'history_window_sec({self._window})가 '
                f'max_observation_sec({self._max_obs}) 이하입니다. '
                '가장 오래된 표본이 잘려 요청 구간을 온전히 못 봅니다.')

        # (timestamp, fire_detected: bool, best_fire_score: float)
        self._history = deque()
        self._labels_seen = set()
        self._warned_near_miss = False

        # ---------------- 배선 ----------------
        self.create_subscription(
            Detection2DArray, self._topic, self.on_detections,
            _sensor_qos(int(p('qos_depth').value))
        )
        self.create_service(
            CheckFireStatus, str(p('service_name').value),
            self.on_check_fire_status
        )

        # ---------------- 통계 ----------------
        # min_score 를 실측으로 정하기 위한 점수 분포 로그.
        self._n_frames = 0
        self._n_fire_frames = 0
        self._scores = []
        stats_period = float(p('stats_period_sec').value)
        if stats_period > 0.0:
            self.create_timer(stats_period, self._report)

        self._event_logger.info(
            f'fire_status_service_node 준비 완료 '
            f'(구독: {self._topic}, 서비스: {p("service_name").value}) | '
            f'fire_class={self._fire_class!r} 창={self._window:.1f}s '
            f'최대관찰={self._max_obs:.1f}s 임계={self._ratio_threshold:.2f} '
            f'min_score={self._min_score:.2f}'
        )

    # ------------------------------------------------------------------ 시계

    def _now(self) -> float:
        """★ time.time()(벽시계)이 아니라 ROS 시계. use_sim_time 을 따른다."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ 구독

    def on_detections(self, msg: Detection2DArray):
        """★ 검출 0개인 빈 배열도 그대로 기록한다 - 그게 '불 없음'이다."""
        now = self._now()
        fire_detected = False
        best_score = 0.0

        for det in msg.detections:
            for res in getattr(det, 'results', ()):
                class_id, score = hypothesis(res)
                self._labels_seen.add(class_id)
                if class_id != self._fire_class:
                    continue
                self._scores.append(score)
                if score < self._min_score:
                    continue
                fire_detected = True
                best_score = max(best_score, score)

        self._history.append((now, fire_detected, best_score))
        self._trim_history(now)

        self._n_frames += 1
        if fire_detected:
            self._n_fire_frames += 1
        self._warn_near_miss_once()

    def _trim_history(self, now: float):
        while self._history and (now - self._history[0][0]) > self._window:
            self._history.popleft()

    def _warn_near_miss_once(self):
        """대소문자만 다른 라벨이 오면 한 번 크게 경고한다.

        놓치면 검출은 멀쩡히 오는데 화재만 0건이라, 불이 타는 중에도
        '꺼짐'으로 응답한다. 예외가 안 나는 종류라 경고가 유일한 방어다.
        자동 교정은 하지 않는다 - 조용히 고치면 원인이 숨는다.
        """
        if self._warned_near_miss:
            return
        target = self._fire_class.casefold()
        near = sorted(l for l in self._labels_seen
                      if l != self._fire_class and l.casefold() == target)
        if near:
            self._warned_near_miss = True
            self.get_logger().error(
                f'fire_class_name={self._fire_class!r} 인데 대소문자만 다른 '
                f'라벨이 들어옵니다: {near}. 화재가 영영 0건으로 잡혀 '
                f"'꺼짐'으로 응답하게 됩니다. 파라미터를 고치세요.")

    # ------------------------------------------------------------------ 서비스

    def on_check_fire_status(self, request, response):
        now = self._now()
        self._trim_history(now)

        observation = float(request.observation_seconds)
        if observation <= 0.0:
            observation = self._default_obs
        if observation > self._max_obs:
            # ★ 조용히 자르지 않는다. 3초를 요청하고 1초 결과를 받는 상황을
            #   호출자가 알아챌 방법이 로그밖에 없다.
            self._event_logger.warn(
                f'observation_seconds={observation:.1f}s 가 최대값 '
                f'{self._max_obs:.1f}s 를 넘어 잘렸습니다.')
            observation = self._max_obs

        window_start = now - observation
        recent = [h for h in self._history if h[0] >= window_start]

        if not recent:
            # 최근 관찰 구간에 아무 데이터도 없으면 판단 불가
            # -> 안전 쪽(안꺼짐)으로 응답한다.
            # ★ 여기서 True 를 내면 카메라가 빠져도 '진압 성공'이 된다.
            #   confidence 0.0 은 "판정 불가" 신호로 쓴다 (응답 필드가
            #   bool+float 뿐이라 상태를 따로 실을 자리가 없다).
            self._event_logger.warn(
                f'판정 불가 - 최근 {observation:.1f}초에 표본 0건. '
                f'{self._topic} 이 오는지, QoS 가 맞는지, yolo_node 가 살아있는지 '
                '확인하세요 (안꺼짐으로 응답)')
            response.is_extinguished = False
            response.confidence = 0.0
            return response

        fire_count = sum(1 for _, detected, _ in recent if detected)
        ratio = fire_count / len(recent)
        is_extinguished = ratio < self._ratio_threshold

        # ★ confidence 는 **판정 자체의 확신도**다 (판정과 같은 방향).
        #   예전처럼 평균 불꽃 점수를 내면 '꺼짐'일 때 값이 0에 수렴해서
        #   로그가 "꺼짐 (확신도 0.00)" 으로 찍혀 자기모순처럼 보인다.
        confidence = (1.0 - ratio) if is_extinguished else ratio

        response.is_extinguished = is_extinguished
        response.confidence = confidence

        fire_scores = [s for _, detected, s in recent if detected]
        mean_score = sum(fire_scores) / len(fire_scores) if fire_scores else 0.0
        self._event_logger.info(
            f'{len(recent)}건 관찰({observation:.1f}s), 불꽃 감지 비율 '
            f'{ratio:.2f} (평균 점수 {mean_score:.2f}) -> '
            f'{"꺼짐" if is_extinguished else "안꺼짐"} (확신도 {confidence:.2f})'
        )
        return response

    # ------------------------------------------------------------------ 통계

    def _report(self):
        """min_score 를 정하기 위한 실측 로그.

        여기 찍히는 score 분포를 보고 min_score 를 정한다. 오검출이
        0.3 언저리에 몰려 있으면 그 위로 자르면 된다.
        """
        if not self._n_frames:
            self._event_logger.warn(
                f'입력 0건 - {self._topic} 이 오지 않습니다. '
                'yolo_node 가 떠 있는지, QoS(BEST_EFFORT)가 맞는지 확인하세요')
            return

        if self._scores:
            lo = min(self._scores)
            hi = max(self._scores)
            avg = sum(self._scores) / len(self._scores)
            dist = f'score min={lo:.2f} 평균={avg:.2f} max={hi:.2f} ({len(self._scores)}건)'
        else:
            dist = 'score 표본 없음'

        self._event_logger.info(
            f'프레임 {self._n_frames}건 (화재 {self._n_fire_frames}건) | '
            f'히스토리 {len(self._history)}건 | {dist}')
        self._n_frames = 0
        self._n_fire_frames = 0
        self._scores = []


def main(args=None):
    rclpy.init(args=args)
    node = FireStatusServiceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ros2 launch 가 SIGINT 를 보내면 rclpy 가 컨텍스트를 먼저 내리고
        # spin 에서 ExternalShutdownException 이 올라온다. 잡아주지 않으면
        # 정상 종료인데도 스택트레이스가 찍혀 진짜 오류와 구분이 안 된다.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()