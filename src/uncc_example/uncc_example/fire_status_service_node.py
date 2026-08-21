#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fire_status_service_node

YOLO 팀이 화재 전용 모델로 띄우는 yolo_detect 두 번째 인스턴스가 발행하는
/yolo_result_fire (타입: interfaces/msg/ObjectsInfo) 토픽을 구독해서
최근 감지 이력을 쌓아두고, CheckFireStatus 서비스 요청이 오면 그 이력을
집계해서 꺼졌는지 판별해 응답한다.

yolov5_ros2 패키지 코드는 전혀 건드리지 않는다 - 이 노드가 결과만 구독한다.

주의: FIRE_CLASS_NAME 값은 YOLO 팀이 실제 학습에 쓴 class_name 문자열과
정확히 일치해야 한다. (예: 데이터셋 라벨이 'fire'가 아니라 'Fire'거나
'flame'일 수 있으니 반드시 확인)
"""

import time
from collections import deque

import rclpy
from rclpy.node import Node

from interfaces.msg import ObjectsInfo
from interfaces.srv import CheckFireStatus

from .log_utils import make_event_logger

FIRE_DETECTION_TOPIC = 'yolo_result_fire'   # YOLO 팀의 화재 모델 노드가 발행하는 토픽
FIRE_CLASS_NAME = 'fire'                     # TODO: YOLO 팀이 실제 쓴 라벨 문자열로 확인/수정
HISTORY_WINDOW_SECONDS = 5.0                 # 이보다 오래된 감지 기록은 버림
EXTINGUISHED_RATIO_THRESHOLD = 0.3            # 관찰 구간 내 불꽃 감지 비율이 이보다 낮으면 '꺼짐'


class FireStatusServiceNode(Node):
    def __init__(self):
        super().__init__('fire_status_service_node')

        self._event_logger = make_event_logger(self)

        self._history = deque()  # (timestamp, fire_detected: bool, score: float)

        self.create_subscription(
            ObjectsInfo, FIRE_DETECTION_TOPIC, self.on_objects_info, 10
        )
        self.create_service(
            CheckFireStatus, 'check_fire_status', self.on_check_fire_status
        )

        self.get_logger().info(
            f'👁️ fire_status_service_node 준비 완료 '
            f'(구독: {FIRE_DETECTION_TOPIC}, 서비스: check_fire_status)'
        )

    def on_objects_info(self, msg: ObjectsInfo):
        now = time.time()
        fire_detected = False
        best_score = 0.0

        for obj in msg.objects:
            if obj.class_name == FIRE_CLASS_NAME:
                fire_detected = True
                best_score = max(best_score, obj.score)

        self._history.append((now, fire_detected, best_score))
        self._trim_history(now)

    def _trim_history(self, now: float):
        while self._history and (now - self._history[0][0]) > HISTORY_WINDOW_SECONDS:
            self._history.popleft()

    def on_check_fire_status(self, request, response):
        now = time.time()
        self._trim_history(now)

        window_start = now - request.observation_seconds
        recent = [h for h in self._history if h[0] >= window_start]

        if not recent:
            # 최근 관찰 구간에 아무 데이터도 없으면 판단 불가 -> 안전 쪽(안꺼짐)으로 응답
            self.get_logger().warn('관찰 구간 내 YOLO 감지 기록이 없습니다. 안꺼짐으로 간주합니다.')
            response.is_extinguished = False
            response.confidence = 0.0
            return response

        fire_count = sum(1 for _, detected, _ in recent if detected)
        ratio = fire_count / len(recent)
        response.is_extinguished = ratio < EXTINGUISHED_RATIO_THRESHOLD
        response.confidence = sum(s for _, _, s in recent) / len(recent)

        self._event_logger.info(
            f'{len(recent)}건 관찰, 불꽃 감지 비율 {ratio:.2f} -> '
            f'{"꺼짐" if response.is_extinguished else "안꺼짐"}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FireStatusServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
