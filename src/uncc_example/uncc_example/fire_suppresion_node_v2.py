#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
화재진압 Action 서버.

- SLAM(frontier_exploration_ros2)의 기존 control_exploration 서비스를
  ACTION_STOP/ACTION_START로 호출해 탐사를 일시 정지/재개한다.
- YOLO 상태는 fire_status_service_node가 노출하는 check_fire_status
  서비스를 통해 확인한다.
- 펌프(GPIO13)/서보(GPIO18)는 라즈베리파이 GPIO를 직접 제어한다.

시퀀스 다이어그램 대응:
  ① 시작 goal 수신           -> execute_callback 진입
  loop (최대 max_attempts회)
    진압 로직 수행            -> run_suppression_routine()
    ② 상태 확인 요청          -> check_fire_status_async()
    ③ 판별 결과 회신 수신     -> status.is_extinguished
    alt 꺼짐 -> ④ SUCCESS 반환 후 종료
       안꺼짐 & 횟수 남음 -> 대기 후 재시도
       안꺼짐 & 횟수 소진 -> ⑤ FAIL 반환
"""

import asyncio

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle

from interfaces.action import SuppressFire
from interfaces.srv import CheckFireStatus
from frontier_exploration_ros2.srv import ControlExploration

from gpiozero import PWMOutputDevice, AngularServo, Device
from gpiozero.pins.lgpio import LGPIOFactory

Device.pin_factory = LGPIOFactory()

# ---------------- 하드웨어 설정 ----------------
PUMP_PIN = 13
SERVO_PIN = 18
PWM_FREQUENCY = 1000

SPRAY_SECONDS = 4.0          # 한 번 진압 시도(1회)에서 분사하는 시간
SERVO_CENTER_ANGLE = 90.0    # 노즐 중앙 각도
SERVO_SWEEP_RANGE_DEG = 20.0  # 중앙 기준 좌우로 움직이는 폭 (총 스윙 = 이 값의 2배)
RETRY_WAIT_SECONDS = 2.0
DEFAULT_MAX_ATTEMPTS = 3
STATUS_CHECK_SECONDS = 2.0

# ControlExploration 서비스는 frontier_exploration_ros2가 이미 제공 (수정 불필요)
CONTROL_EXPLORATION_SERVICE = 'control_exploration'
CHECK_FIRE_STATUS_SERVICE = 'check_fire_status'


class FireSuppressionNode(Node):
    def __init__(self):
        super().__init__('fire_suppression_node')

        self.pump = PWMOutputDevice(
            PUMP_PIN, active_high=False, frequency=PWM_FREQUENCY, initial_value=0.0
        )
        self.servo = AngularServo(
            SERVO_PIN, min_angle=0, max_angle=180,
            min_pulse_width=0.0005, max_pulse_width=0.0025,
        )

        self.yolo_client = self.create_client(CheckFireStatus, CHECK_FIRE_STATUS_SERVICE)
        self.exploration_client = self.create_client(
            ControlExploration, CONTROL_EXPLORATION_SERVICE
        )

        self._action_server = ActionServer(
            self,
            SuppressFire,
            'suppress_fire',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('🔥 화재진압 노드 준비 완료 (Action: suppress_fire)')

    def cancel_callback(self, goal_handle):
        """취소 요청을 받아들일지 결정. rclpy 기본값은 REJECT라서
        명시적으로 ACCEPT 해주지 않으면 취소 요청이 계속 씹힌다."""
        self.get_logger().info('취소 요청 수신, 다음 안전 시점에 정지합니다.')
        return CancelResponse.ACCEPT

    # ---------------- 탐사 제어 (기존 서비스 재사용) ----------------
    async def set_exploration_running(self, running: bool):
        if not self.exploration_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                'control_exploration 서비스에 연결할 수 없습니다. 탐사 상태를 건드리지 않고 진행합니다.'
            )
            return

        req = ControlExploration.Request()
        req.action = (
            ControlExploration.Request.ACTION_START
            if running
            else ControlExploration.Request.ACTION_STOP
        )
        req.delay_seconds = 0.0
        req.quit_after_stop = False

        future = self.exploration_client.call_async(req)
        response = await future
        self.get_logger().info(
            f'탐사 {"재개" if running else "정지"} 요청: accepted={response.accepted}, '
            f'message={response.message}'
        )

    # ---------------- 취소 지원 ----------------
    async def interruptible_sleep(self, goal_handle, seconds: float, poll_interval: float = 0.1):
        """일반 asyncio.sleep과 달리 취소 요청이 오면 즉시 빠져나온다.
        반환값 True면 도중에 취소된 것."""
        elapsed = 0.0
        while elapsed < seconds:
            if goal_handle.is_cancel_requested:
                return True
            step = min(poll_interval, seconds - elapsed)
            await asyncio.sleep(step)
            elapsed += step
        return goal_handle.is_cancel_requested

    # ---------------- 하드웨어 제어 ----------------
    async def run_suppression_routine(self, goal_handle):
        """펌프 ON + 서보를 중앙 기준 좌우 SERVO_SWEEP_RANGE_DEG도로 왕복
        스윕하며 SPRAY_SECONDS 동안 분사 후 정지 (= 1회 진압 시도).
        도중에 취소 요청이 오면 스윕을 중단하고 즉시 펌프를 끈다.
        반환값 True면 스프레이 도중 취소된 것."""
        left = SERVO_CENTER_ANGLE - SERVO_SWEEP_RANGE_DEG
        right = SERVO_CENTER_ANGLE + SERVO_SWEEP_RANGE_DEG
        angles = [left, right, left, right]  # 4초 동안 좌우 왕복 2회
        step = SPRAY_SECONDS / len(angles)

        self.pump.value = 1.0
        cancelled = False
        for a in angles:
            self.servo.angle = a
            cancelled = await self.interruptible_sleep(goal_handle, step)
            if cancelled:
                break

        self.pump.value = 0.0
        self.servo.angle = SERVO_CENTER_ANGLE
        return cancelled

    # ---------------- YOLO 상태 확인 ----------------
    async def check_fire_status_async(self, observation_seconds: float):
        if not self.yolo_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('check_fire_status 서비스에 연결할 수 없습니다.')
            return None

        req = CheckFireStatus.Request()
        req.observation_seconds = observation_seconds

        future = self.yolo_client.call_async(req)
        response = await future
        return response

    # ---------------- Action 실행 ----------------
    async def execute_callback(self, goal_handle: ServerGoalHandle):
        max_attempts = goal_handle.request.max_attempts or DEFAULT_MAX_ATTEMPTS
        feedback_msg = SuppressFire.Feedback()
        result = SuppressFire.Result()

        self.get_logger().info(f'진압 시작. 최대 {max_attempts}회 시도.')
        await self.set_exploration_running(False)

        try:
            for attempt in range(1, max_attempts + 1):
                if goal_handle.is_cancel_requested:
                    return self._handle_cancel(goal_handle, attempt - 1)

                feedback_msg.current_attempt = attempt
                feedback_msg.status = f'{attempt}차 진압 로직 수행 중'
                goal_handle.publish_feedback(feedback_msg)

                cancelled = await self.run_suppression_routine(goal_handle)
                if cancelled:
                    return self._handle_cancel(goal_handle, attempt)

                feedback_msg.status = f'{attempt}차 상태 확인 중'
                goal_handle.publish_feedback(feedback_msg)

                status = await self.check_fire_status_async(STATUS_CHECK_SECONDS)

                if status is None:
                    self.get_logger().warn('YOLO 응답 없음. 이번 시도는 실패로 간주하고 재시도.')
                    is_extinguished = False
                else:
                    is_extinguished = status.is_extinguished
                    self.get_logger().info(
                        f'{attempt}차 판별 결과: {"꺼짐" if is_extinguished else "안꺼짐"} '
                        f'(확신도 {status.confidence:.2f})'
                    )

                if is_extinguished:
                    goal_handle.succeed()
                    result.success = True
                    result.attempts = attempt
                    result.message = '진압 성공'
                    return result

                if attempt < max_attempts:
                    cancelled = await self.interruptible_sleep(goal_handle, RETRY_WAIT_SECONDS)
                    if cancelled:
                        return self._handle_cancel(goal_handle, attempt)

            goal_handle.succeed()
            result.success = False
            result.attempts = max_attempts
            result.message = '진압 실패 (최대 시도 횟수 도달)'
            return result

        finally:
            await self.set_exploration_running(True)

    def _handle_cancel(self, goal_handle, attempts_done: int):
        """취소 확정 처리: 펌프 즉시 정지, 서보 중앙 복귀, 취소 결과 반환.
        set_exploration_running(True)는 execute_callback의 finally에서 처리됨."""
        self.pump.value = 0.0
        self.servo.angle = SERVO_CENTER_ANGLE
        goal_handle.canceled()

        result = SuppressFire.Result()
        result.success = False
        result.attempts = attempts_done
        result.message = '진압 취소됨 (사용자 요청)'
        self.get_logger().info(f'진압 작업이 취소되었습니다 ({attempts_done}회 시도 후). 펌프 정지.')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FireSuppressionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pump.off()
        node.servo.detach()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
