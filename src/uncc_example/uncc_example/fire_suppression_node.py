#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
화재진압 Action 서버.

- SLAM(frontier_exploration_ros2)의 기존 control_exploration 서비스를
  ACTION_STOP/ACTION_START로 호출해 탐사를 일시 정지/재개한다.
- YOLO 상태는 fire_status_service_node가 노출하는 check_fire_status
  서비스를 통해 확인한다.
- 펌프(GPIO13)/서보(GPIO18)는 라즈베리파이 GPIO를 직접 제어한다.
- 분사 후 재시도 대기 구간에서는 서보 PWM 신호를 detach()로 끊어서
  떨림/웅- 소음을 없앤다 (테스트 스크립트에서 검증된 패턴 반영).
- 스윕 자체는 EMA 스무딩/가변속 없이 min/max 각도를 단순 왕복하는
  방식으로 단순화함 (저가형 서보에서는 정밀 스무딩 효과가 미미했음).

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

import signal

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from rclpy.task import Future

from interfaces.action import SuppressFire
from interfaces.srv import CheckFireStatus
from frontier_exploration_ros2.srv import ControlExploration

from gpiozero import PWMOutputDevice, AngularServo, Device
from gpiozero.pins.lgpio import LGPIOFactory

Device.pin_factory = LGPIOFactory()

# 하드웨어 설정
PUMP_PIN = 13
SERVO_PIN = 18
PWM_FREQUENCY = 1000

SPRAY_SECONDS = 3.0
RETRY_WAIT_SECONDS = 2.0
DEFAULT_MAX_ATTEMPTS = 3
STATUS_CHECK_SECONDS = 3.0

# 서보 스윕 설정 (단순 왕복 방식)
SERVO_CENTER_ANGLE = 90
SERVO_SWEEP_RANGE_DEG = 15               # 중앙 기준 +-범위
SERVO_SWEEP_STEP_SECONDS = 0.6           # 한쪽 끝에서 반대쪽 끝까지 이동하는 데 걸리는 시간

# 서보 펄스폭 (테스트 스크립트에서 확인된 안전 범위로 조정: 0.0005~0.0025 -> 0.0006~0.0024)
SERVO_MIN_PULSE_WIDTH = 0.0006
SERVO_MAX_PULSE_WIDTH = 0.0024

# 대기 구간 진입 시, 중앙으로 복귀한 뒤 detach()하기 전까지 대기하는 시간(초)
# 너무 짧으면 물리적으로 중앙에 도달하기 전에 신호가 끊겨 어중간한 위치에서 멈출 수 있음
SERVO_SETTLE_SECONDS = 0.3

# _rclpy_sleep 이 공유해서 쓰는 재사용 타이머의 tick 주기(초).
# 매번 create_timer/destroy_timer 를 새로 하지 않기 위해, 이 주기로 도는
# 타이머 하나만 만들어두고 남은 틱 수를 세는 방식으로 sleep 을 구현한다.
SLEEP_TICK_SECONDS = 0.05

# ControlExploration 서비스는 frontier_exploration_ros2가 이미 제공 (수정 불필요)
CONTROL_EXPLORATION_SERVICE = 'control_exploration'
CHECK_FIRE_STATUS_SERVICE = 'check_fire_status'


class FireSuppressionNode(Node):
    def __init__(self):
        super().__init__('fire_suppression_node')
        self._busy = False

        self.pump = PWMOutputDevice(
            PUMP_PIN, active_high=False, frequency=PWM_FREQUENCY, initial_value=0.0
        )
        self.servo = AngularServo(
            SERVO_PIN, min_angle=0, max_angle=180,
            min_pulse_width=SERVO_MIN_PULSE_WIDTH, max_pulse_width=SERVO_MAX_PULSE_WIDTH,
            initial_angle=None,
        )

        self.yolo_client = self.create_client(CheckFireStatus, CHECK_FIRE_STATUS_SERVICE)
        self.exploration_client = self.create_client(
            ControlExploration, CONTROL_EXPLORATION_SERVICE
        )

        # _rclpy_sleep 전용 재사용 타이머. 매 sleep 마다 타이머를 새로
        # 만들고 없애던 방식(과거) 대신, 이 타이머 하나를 계속 돌리면서
        # 남은 틱 수(_sleep_ticks_remaining)를 세는 방식으로 바꿨다.
        # 액션 콜백은 한 번에 하나만 실행되므로 동시에 두 개의 sleep이
        # 겹칠 일은 없다고 가정한다.
        self._sleep_future = None
        self._sleep_ticks_remaining = 0
        self._sleep_timer = self.create_timer(SLEEP_TICK_SECONDS, self._on_sleep_tick)

        self._action_server = ActionServer(
            self,
            SuppressFire,
            'suppress_fire',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info('🔥 화재진압 노드 준비 완료 (Action: suppress_fire)')

    def goal_callback(self, goal_request):
        """이미 진압 중이면 새 goal을 거부한다. 이게 없으면 rclpy가
        goal을 무조건 다 받아서 execute_callback이 동시에 여러 개
        돌 수 있고, 그러면 servo/pump/sleep 상태를 여러 코루틴이
        동시에 건드려서 꼬인다."""
        if self._busy:
            self.get_logger().warn('이미 진압 작업 실행 중 — 새 goal 거부')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

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
    def _on_sleep_tick(self):
        """재사용 타이머의 콜백. 대기 중인 sleep 이 있으면 남은 틱을
        하나 소모하고, 다 소모됐으면 Future 를 완료시켜 깨운다."""
        if self._sleep_future is None:
            return
        self._sleep_ticks_remaining -= 1
        if self._sleep_ticks_remaining <= 0 and not self._sleep_future.done():
            self._sleep_future.set_result(None)

    async def _rclpy_sleep(self, seconds: float):
        """asyncio.sleep 대체용. rclpy 액션 콜백 안에는 진짜 asyncio
        이벤트루프가 없어서 asyncio.sleep()이 'no running event loop'로
        터진다. 매번 새 타이머를 만들지 않고, 노드 생성 시 만들어둔
        재사용 타이머(self._sleep_timer)의 tick 수를 세는 방식으로
        rclpy.task.Future 를 완료시켜 안전하게 await 한다."""
        ticks = max(1, round(seconds / SLEEP_TICK_SECONDS))
        self._sleep_ticks_remaining = ticks
        self._sleep_future = Future()
        try:
            await self._sleep_future
        finally:
            self._sleep_future = None

    async def interruptible_sleep(self, goal_handle, seconds: float, poll_interval: float = 0.1):
        """일반 sleep과 달리 취소 요청이 오면 즉시 빠져나온다.
        반환값 True면 도중에 취소된 것."""
        elapsed = 0.0
        while elapsed < seconds:
            if goal_handle.is_cancel_requested:
                return True
            step = min(poll_interval, seconds - elapsed)
            await self._rclpy_sleep(step)
            elapsed += step
        return goal_handle.is_cancel_requested

    # ---------------- 하드웨어 제어 ----------------
    async def _settle_and_detach(self):
        """서보를 중앙으로 복귀시키고, 물리적으로 도달할 시간을 준 뒤
        PWM 신호를 끊는다. 대기 구간 동안 서보가 떨거나 웅- 소음을
        내는 걸 막아준다 (테스트 스크립트에서 검증된 패턴)."""
        self.servo.angle = SERVO_CENTER_ANGLE
        await self._rclpy_sleep(SERVO_SETTLE_SECONDS)
        self.servo.detach()

    async def run_suppression_routine(self, goal_handle) -> bool:
        """펌프 ON 상태로 SPRAY_SECONDS 동안 min/max 각도를 단순 왕복하며
        분사한다 (EMA 스무딩/가변속 스윕은 저가형 서보에서 체감 효과가
        없어 제거하고 단순 왕복으로 변경).
        끝나면(취소든 정상 종료든) 중앙 복귀 후 detach()로 정숙 대기
        상태에 들어간다. 도중에 취소 요청이 오면 즉시 정지하고 True를
        반환한다."""
        min_angle = SERVO_CENTER_ANGLE - SERVO_SWEEP_RANGE_DEG
        max_angle = SERVO_CENTER_ANGLE + SERVO_SWEEP_RANGE_DEG

        angle = min_angle
        self.servo.angle = angle  # detach 상태였다면 여기서 자동 재개됨

        self.pump.value = 1.0
        elapsed = 0.0
        try:
            while elapsed < SPRAY_SECONDS:
                if goal_handle.is_cancel_requested:
                    return True
                angle = max_angle if angle == min_angle else min_angle
                self.servo.angle = angle
                await self._rclpy_sleep(SERVO_SWEEP_STEP_SECONDS)
                elapsed += SERVO_SWEEP_STEP_SECONDS
        finally:
            self.pump.value = 0.0
            await self._settle_and_detach()

        return False

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
        self._busy = True
        try:
            max_attempts = goal_handle.request.max_attempts or DEFAULT_MAX_ATTEMPTS
            feedback_msg = SuppressFire.Feedback()
            result = SuppressFire.Result()

            self.get_logger().info(f'진압 시작. 최대 {max_attempts}회 시도.')
            await self.set_exploration_running(False)

            try:
                for attempt in range(1, max_attempts + 1):
                    if goal_handle.is_cancel_requested:
                        return await self._handle_cancel(goal_handle, attempt - 1)

                    feedback_msg.current_attempt = attempt
                    feedback_msg.status = f'{attempt}차 진압 로직 수행 중'
                    goal_handle.publish_feedback(feedback_msg)

                    cancelled = await self.run_suppression_routine(goal_handle)
                    if cancelled:
                        return await self._handle_cancel(goal_handle, attempt)

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
                            return await self._handle_cancel(goal_handle, attempt)

                goal_handle.succeed()
                result.success = False
                result.attempts = max_attempts
                result.message = '진압 실패 (최대 시도 횟수 도달)'
                return result

            finally:
                await self.set_exploration_running(True)

        finally:
            self._busy = False

    async def _handle_cancel(self, goal_handle, attempts_done: int):
        """취소 확정 처리: 펌프 즉시 정지, 서보 중앙 복귀 후 detach, 취소 결과 반환.
        set_exploration_running(True)는 execute_callback의 finally에서 처리됨."""
        self.pump.value = 0.0
        await self._settle_and_detach()
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

    # ros2 launch 가 SIGINT 이후에도 종료가 늦어지면 SIGTERM 으로
    # 강제 종료시키는 경우가 있다. 파이썬 기본 동작은 SIGTERM 을 받으면
    # try/finally 도 안 거치고 그냥 죽어버리므로, SIGTERM 을
    # KeyboardInterrupt 로 변환해서 아래 finally(하드웨어 정리)가
    # 항상 실행되도록 만든다.
    def _handle_sigterm(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _handle_sigterm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # off()/detach() 는 신호만 끊을 뿐 GPIO 핀(lgpio 칩 핸들) 자체는
        # 계속 점유한 상태로 남는다. close() 를 호출해 핀을 완전히
        # 반환해야 다음 재실행 때 핀 점유 상태가 깨끗하게 시작된다.
        node.pump.close()
        node.servo.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()