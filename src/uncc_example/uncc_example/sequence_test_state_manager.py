"""비전 없이 고정 좌표 2개(불1 -> 사람)를 순서대로 도는 테스트 전용
상태 관리자.

시나리오: start_mission -> 좌우 스캔(DemoStateManager 재사용) -> 불1 진압 ->
사람 좌표 도착 확인 -> base 복귀.

/vision/detections 는 완전히 무시한다 — 좌표는 launch 파라미터
(fire1_xy/person_xy)로 고정해서 받고, 거리 기반 클러스터링/우선순위
로직(target_queue) 없이 위 순서를 그대로 강제한다.
"""

import time

import rclpy

from .demo_state_manager import DemoStateManager


class SequenceTestStateManager(DemoStateManager):

    PHASE_FIRE1 = 'test_fire1'
    PHASE_PERSON = 'test_person'

    def __init__(self):
        super().__init__()

        self.declare_parameter('fire1_xy', [1.0, 2.0])
        self.declare_parameter('person_xy', [1.5, 1.5])

        fire1_xy = self.get_parameter('fire1_xy').value
        person_xy = self.get_parameter('person_xy').value

        self._fire1 = self._make_test_entry('fire', fire1_xy)
        self._person = self._make_test_entry('person', person_xy)

        # fire_keepout_node/map_visualizer/UI 가 처음부터 볼 수 있게 미리
        # found_targets 에 채워둔다 (target_queue 는 이 테스트에서 안 씀).
        self.found_targets = [self._fire1, self._person]
        self._publish_found_targets()

        self.get_logger().info(
            'Sequence test ready: sweep -> '
            f'fire1{tuple(fire1_xy)} -> person{tuple(person_xy)} -> base'
        )

    def _make_test_entry(self, target_type, xy):
        pose = self._make_pose_stamped(self.map_frame, float(xy[0]), float(xy[1]))
        return {
            'type': target_type,
            'pose': pose,
            'status': 'pending',
            'cluster': None,
            'extinguished': None,
            'unreachable': False,
        }

    # =========================================================
    # /vision/detections 는 이 테스트에서 완전히 무시한다.
    # =========================================================

    def detection_callback(self, msg):
        return

    # =========================================================
    # State selection — 고정 순서(fire1 -> person -> base)
    # =========================================================

    def _refresh_state(self):
        if not self._mission_started:
            self._enter_standby()
            return

        if self._manual_stop:
            self.phase = self.PHASE_MANUAL_RETURN
            self._enter_returning_when_pose_ready('manual stop')
            return

        if self.phase == self.PHASE_COMPLETE:
            self._enter_terminal_state(self.MISSION_COMPLETE)
            return

        if self.phase == self.PHASE_FAILED:
            self._enter_terminal_state(self.MISSION_FAILED)
            return

        if self.is_battery_low():
            self.phase = self.PHASE_FINAL_RETURN
            self._enter_returning_when_pose_ready('low battery')
            return

        now = time.monotonic()

        if self.phase == self.PHASE_INITIAL_SWEEP:
            self._process_initial_sweep(now)
        elif self.phase == self.PHASE_FIRE1:
            self._enter_urgent_target(self._fire1)
        elif self.phase == self.PHASE_PERSON:
            self._enter_urgent_target(self._person)
        elif self.phase == self.PHASE_FINAL_RETURN:
            self._enter_returning_when_pose_ready('sequence complete')
        else:
            self._enter_terminal_state(self.MISSION_FAILED)

    def _process_initial_sweep(self, now):
        """DemoStateManager 와 동일하되, 스캔이 끝나면 군집 판단 없이
        곧장 불1로 넘어간다."""

        self._enter_terminal_state(self.INITIAL_SWEEP)

        if self._spin_pending:
            return

        if self._dwell_until is not None and now < self._dwell_until:
            return

        self._dwell_until = None

        if self._sweep_step_index < len(self._sweep_steps):
            angle = self._sweep_steps[self._sweep_step_index]
            self._send_spin(angle, 'initial_sweep')
            return

        self.phase = self.PHASE_FIRE1
        self._event_logger.info('초기 좌우 스캔 완료: 불1 좌표로 이동 시작')

    # =========================================================
    # Target completion — 다음 phase 로 넘긴다 (target_queue 미사용)
    # =========================================================

    def target_complete_callback(self, request, response):
        if self.state == self.RETURNING_TO_CHARGE and self.active_target is None:
            if self.phase == self.PHASE_FINAL_RETURN:
                self.phase = self.PHASE_COMPLETE
                self._event_logger.info('최종 복귀 완료: 시퀀스 테스트 종료')
            elif self.phase == self.PHASE_MANUAL_RETURN:
                self._reset_demo_after_manual_stop()

            response.success = True
            return response

        entry = self.active_target

        if entry is not None:
            entry['status'] = 'done'
            entry['extinguished'] = (request.data == self.TARGET_STATUS_SUCCESS)
            entry['unreachable'] = (request.data == self.TARGET_STATUS_UNREACHABLE)
            self._publish_found_targets()

        self.active_target = None

        if self.phase == self.PHASE_FIRE1:
            self.phase = self.PHASE_PERSON
            self._event_logger.info('불1 처리 완료: 사람 좌표로 이동')
        elif self.phase == self.PHASE_PERSON:
            self.phase = self.PHASE_FINAL_RETURN
            self._event_logger.info('사람 좌표 도착 확인: base로 최종 복귀')

        response.success = True
        return response

    def _reset_demo_after_manual_stop(self):
        """stop_mission 뒤 다시 start_mission 하면 세 좌표를 처음
        상태(pending)로 되돌려서 반복 테스트가 가능하게 한다."""

        super()._reset_demo_after_manual_stop()

        for entry in (self._fire1, self._person):
            entry['status'] = 'pending'
            entry['extinguished'] = None
            entry['unreachable'] = False

        self.found_targets = [self._fire1, self._person]
        self._publish_found_targets()


def main(args=None):
    rclpy.init(args=args)
    node = SequenceTestStateManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
