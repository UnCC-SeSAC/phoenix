#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
진압 거리 테스트용 노드 — 탐사/Nav2 없이 **오도메트리 기준 N cm 전진 후
suppress_fire 액션 호출**만 한다.

    /odom 로 이동거리 측정
        │
        ▼
    /cmd_vel 직접 발행 ──▶ odom_publisher(controller) ──▶ 모터
        │
        │ 목표거리 도달 → 정지 → 정착 대기
        ▼
    suppress_fire 액션 goal ──▶ fire_suppression_node (펌프/서보 실제 구동)

왜 Nav2 를 안 쓰나
------------------
Nav2 는 goal 근처에서 xy_goal_tolerance(0.25 m) 안에 들어오면 멈추므로,
"정확히 10 cm 앞"을 만들 수 없다. 이 노드는 costmap·플래너를 전부 건너뛰고
/cmd_vel 을 직접 발행한다. **그래서 장애물을 전혀 보지 않는다** — 앞이 비어
있는지 눈으로 확인하고 돌릴 것.

왜 오픈루프(시간 x 속도)가 아닌가
---------------------------------
모터 데드밴드와 기동 램프 때문에 "0.08 m/s 로 1.25초"가 실제로 10 cm 가
되지 않는다. 진압 거리를 재는 게 목적인 테스트에서 그 오차는 측정 대상 자체를
망가뜨린다. 그래서 /odom 위치 변화를 적분해 목표거리에서 멈추고, **실제로
움직인 거리를 로그로 남긴다.**

안전장치
--------
  - 워치독: 예상 소요시간의 3배 + 2초가 지나면 목표 미달이어도 정지한다
    (바퀴가 헛돌거나 /odom 이 안 오면 영원히 전진하는 걸 막는다)
  - 종료(Ctrl+C/SIGTERM) 시 0 속도를 여러 번 내보내고 죽는다
  - /odom 이 한 번도 안 오면 **아예 출발하지 않는다**

사용
----
    ros2 launch uncc_example nudge_suppress_test.launch.py

    # 다시 돌리기 (런치 재시작 없이)
    ros2 service call /nudge_and_suppress/run std_srvs/srv/Trigger "{}"
"""

import math
import signal

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger

from interfaces.action import SuppressFire


# 목표거리의 이 비율 안쪽에 들어오면 속도를 절반으로 낮춘다. 저속 구간이
# 없으면 정지 명령 이후의 관성만큼 그대로 오버슛한다.
_SLOWDOWN_FRACTION = 0.3

# 정지 명령을 이만큼 반복 발행한다. 한 번만 보내면 드라이버 쪽에서 유실될 때
# 로봇이 마지막 속도로 계속 간다.
_STOP_REPEAT = 5


class NudgeAndSuppress(Node):

    IDLE = 'IDLE'
    WAIT_ODOM = 'WAIT_ODOM'
    MOVING = 'MOVING'
    STOPPING = 'STOPPING'
    SETTLING = 'SETTLING'
    SUPPRESSING = 'SUPPRESSING'
    DONE = 'DONE'

    def __init__(self):
        super().__init__('nudge_and_suppress')

        # -----------------------------
        # Parameters
        # -----------------------------
        # 음수를 주면 뒤로 간다 (진압 거리를 늘려가며 볼 때 유용).
        self.declare_parameter('distance', 0.10)
        # 모터 데드밴드보다는 위, 오버슛을 만들 만큼은 아래. 실기에서
        # 안 움직이면 0.10~0.12 까지 올릴 것.
        self.declare_parameter('speed', 0.08)
        # 정지 후 진동이 잦아들 때까지 기다렸다가 진압을 시작한다.
        self.declare_parameter('settle_sec', 1.0)
        # 0 이면 fire_suppression_node 의 기본값(3회)을 쓴다.
        self.declare_parameter('max_attempts', 1)
        # 노드가 뜨자마자 자동으로 한 번 돌릴지. false 면 ~/run 서비스로만 시작.
        self.declare_parameter('auto_start', True)
        # 자동 시작 전 대기. controller/EKF 가 /odom 을 안정적으로 낼 때까지.
        self.declare_parameter('start_delay_sec', 3.0)
        # 이동만 하고 진압은 부르지 않는다 (거리 캘리브레이션용).
        self.declare_parameter('skip_suppression', False)
        # 진압 결과를 받은 뒤 런치를 자동 종료할지. false 면 ~/run 으로 반복 가능.
        self.declare_parameter('shutdown_after', False)

        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('control_period', 0.05)

        p = self.get_parameter
        self.distance = float(p('distance').value)
        self.speed = abs(float(p('speed').value))
        self.settle_sec = float(p('settle_sec').value)
        self.max_attempts = int(p('max_attempts').value)
        self.skip_suppression = bool(p('skip_suppression').value)
        self.shutdown_after = bool(p('shutdown_after').value)
        self.control_period = float(p('control_period').value)

        if self.speed <= 0.0:
            raise ValueError('speed 는 0보다 커야 합니다')

        # 워치독: 예상 소요시간의 3배 + 2초. 바퀴가 헛돌거나 /odom 이
        # 멈추면 여기서 끊는다.
        self._move_timeout_sec = abs(self.distance) / self.speed * 3.0 + 2.0

        # -----------------------------
        # State
        # -----------------------------
        self.state = self.IDLE
        self._odom_xy = None          # 최신 /odom 위치 (x, y)
        self._start_xy = None         # 이번 이동의 출발 좌표
        self._phase_started = 0.0     # 현재 단계 진입 시각 (ROS 시계)
        self._stop_ticks = 0
        self._traveled = 0.0          # 이번 이동에서 실제로 간 거리
        self._goal_handle = None
        self._autostarted = False

        # -----------------------------
        # Wiring
        # -----------------------------
        self.cmd_pub = self.create_publisher(
            Twist, str(p('cmd_vel_topic').value), 10)

        self.create_subscription(
            Odometry, str(p('odom_topic').value), self._odom_callback, 10)

        self._action_client = ActionClient(self, SuppressFire, 'suppress_fire')

        self.create_service(Trigger, '~/run', self._run_callback)

        self.create_timer(self.control_period, self._tick)

        self.get_logger().info(
            f'nudge_and_suppress 준비 — 목표 {self.distance * 100:.1f} cm, '
            f'속도 {self.speed:.3f} m/s, 워치독 {self._move_timeout_sec:.1f} s'
        )
        self.get_logger().warn(
            '★ 이 노드는 장애물을 보지 않습니다. 로봇 앞이 비었는지 확인하세요.'
        )

        if bool(p('auto_start').value):
            delay = float(p('start_delay_sec').value)
            self.get_logger().info(f'{delay:.1f}초 뒤 자동 시작합니다.')
            self._autostart_timer = self.create_timer(delay, self._autostart)

    # =========================================================
    # 입력
    # =========================================================

    def _odom_callback(self, msg):
        self._odom_xy = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def _run_callback(self, request, response):
        if self.state not in (self.IDLE, self.DONE):
            response.success = False
            response.message = f'이미 진행 중입니다 (state={self.state})'
            return response

        self._begin()
        response.success = True
        response.message = '시작했습니다'
        return response

    def _autostart(self):
        # 자동 시작은 한 번뿐이다. 자기 콜백 안에서 destroy_timer 를 부르면
        # rclpy 버전에 따라 핸들이 꼬이므로 cancel + 플래그로만 막는다.
        if self._autostarted:
            return
        self._autostarted = True
        self._autostart_timer.cancel()
        self._begin()

    # =========================================================
    # 상태 전이
    # =========================================================

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _enter(self, state):
        self.state = state
        self._phase_started = self._now()

    def _begin(self):
        self._traveled = 0.0
        self._start_xy = None
        self._enter(self.WAIT_ODOM)
        self.get_logger().info('=== 시작: /odom 대기 ===')

    def _tick(self):
        handler = {
            self.WAIT_ODOM: self._tick_wait_odom,
            self.MOVING: self._tick_moving,
            self.STOPPING: self._tick_stopping,
            self.SETTLING: self._tick_settling,
        }.get(self.state)

        if handler is not None:
            handler()

    def _tick_wait_odom(self):
        if self._odom_xy is None:
            # /odom 없이 출발하면 언제 멈춰야 하는지 알 수 없다. 절대 안 나간다.
            self.get_logger().warn(
                f'{self.get_parameter("odom_topic").value} 이 오지 않습니다 — '
                'controller/EKF 가 떠 있는지 확인하세요',
                throttle_duration_sec=2.0,
            )
            return

        self._start_xy = self._odom_xy
        self._enter(self.MOVING)
        self.get_logger().info(
            f'전진 시작 — 출발 ({self._start_xy[0]:.3f}, {self._start_xy[1]:.3f})'
        )

    def _tick_moving(self):
        self._traveled = math.hypot(
            self._odom_xy[0] - self._start_xy[0],
            self._odom_xy[1] - self._start_xy[1],
        )
        target = abs(self.distance)

        if self._traveled >= target:
            self.get_logger().info(
                f'목표 도달 — 이동 {self._traveled * 100:.1f} cm'
            )
            self._enter(self.STOPPING)
            return

        if self._now() - self._phase_started > self._move_timeout_sec:
            self.get_logger().error(
                f'워치독 — {self._move_timeout_sec:.1f}초 안에 목표거리에 '
                f'도달하지 못했습니다 (이동 {self._traveled * 100:.1f} cm). '
                'speed 를 올리거나 바퀴 접지를 확인하세요. 정지합니다.'
            )
            self._enter(self.STOPPING)
            return

        remaining = target - self._traveled
        # 마지막 구간은 절반 속도로 — 정지 명령 뒤 관성 오버슛을 줄인다.
        speed = self.speed
        if remaining < target * _SLOWDOWN_FRACTION:
            speed *= 0.5

        twist = Twist()
        twist.linear.x = speed if self.distance >= 0.0 else -speed
        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f'이동 중 {self._traveled * 100:.1f} / {target * 100:.1f} cm',
            throttle_duration_sec=0.5,
        )

    def _tick_stopping(self):
        self.cmd_pub.publish(Twist())
        self._stop_ticks += 1

        if self._stop_ticks < _STOP_REPEAT:
            return

        self._stop_ticks = 0

        # 정지 명령 이후 실제로 더 간 거리까지 포함해 최종값을 다시 잰다.
        self._traveled = math.hypot(
            self._odom_xy[0] - self._start_xy[0],
            self._odom_xy[1] - self._start_xy[1],
        )
        error_cm = (self._traveled - abs(self.distance)) * 100.0
        self.get_logger().info(
            f'정지 — 최종 이동 {self._traveled * 100:.1f} cm '
            f'(목표 대비 {error_cm:+.1f} cm)'
        )

        if self.skip_suppression:
            self.get_logger().info('skip_suppression=true — 진압을 부르지 않고 종료합니다.')
            self._finish()
            return

        self._enter(self.SETTLING)

    def _tick_settling(self):
        if self._now() - self._phase_started < self.settle_sec:
            return

        self._enter(self.SUPPRESSING)
        self._send_suppression_goal()

    # =========================================================
    # 진압 액션
    # =========================================================

    def _send_suppression_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'suppress_fire 액션 서버가 없습니다 — fire_suppression_node 가 '
                '떠 있는지 확인하세요.'
            )
            self._finish()
            return

        goal = SuppressFire.Goal()
        goal.max_attempts = self.max_attempts

        attempts_text = (
            '서버 기본값' if self.max_attempts == 0
            else f'{self.max_attempts}회'
        )
        self.get_logger().info(f'진압 goal 전송 (최대 시도 {attempts_text})')

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(self._goal_response)

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'  진압 {fb.current_attempt}차 — {fb.status}')

    def _goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'goal 이 거부됐습니다 — 이미 진압 중일 수 있습니다.')
            self._finish()
            return

        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, future):
        self._goal_handle = None
        outcome = future.result()

        if outcome.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('진압이 취소됐습니다.')
            self._finish()
            return

        result = outcome.result
        verdict = '성공' if result.success else '실패'
        self.get_logger().info(
            f'=== 진압 {verdict} — {result.attempts}회 시도, '
            f'"{result.message}" | 이동거리 {self._traveled * 100:.1f} cm ==='
        )
        self._finish()

    # =========================================================
    # 마무리
    # =========================================================

    def _finish(self):
        self._enter(self.DONE)
        self.cmd_pub.publish(Twist())

        if self.shutdown_after:
            self.get_logger().info('shutdown_after=true — 종료합니다.')
            raise SystemExit

        self.get_logger().info(
            '대기 상태입니다. 다시 돌리려면: '
            'ros2 service call /nudge_and_suppress/run std_srvs/srv/Trigger "{}"'
        )

    def stop_motors(self):
        """종료 경로에서 반드시 불린다. 마지막 속도로 계속 가는 걸 막는다."""
        try:
            for _ in range(_STOP_REPEAT):
                self.cmd_pub.publish(Twist())
        except Exception:
            # 컨텍스트가 이미 내려갔으면 더 할 수 있는 게 없다.
            pass


def main(args=None):
    rclpy.init(args=args)
    node = NudgeAndSuppress()

    # ros2 launch 는 SIGINT 이후 종료가 늦으면 SIGTERM 을 보낸다. 파이썬 기본
    # 동작은 SIGTERM 에 finally 도 안 거치고 죽으므로, 바퀴가 돌고 있는 채로
    # 노드만 사라진다. KeyboardInterrupt 로 바꿔 아래 finally 를 태운다.
    def _handle_sigterm(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _handle_sigterm)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
