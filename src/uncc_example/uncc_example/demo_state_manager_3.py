"""단일 fire/person을 처리하고 최종 복귀 뒤 결과를 판정하는 데모.

초기 스캔에서 두 객체가 모두 보이면 fire를 먼저 처리한 뒤 person으로
이동한다. 한 종류만 보이면 그 객체를 먼저 처리하고 base로 돌아와 최초
헤딩으로 정렬한 다음, 누락된 종류를 다시 스캔한다.

객체 처리 실패는 즉시 미션을 중단하지 않는다. fire 진압 결과와 person
도착 결과를 각각 저장하고, 가능한 동작과 최종 base 복귀를 모두 수행한
뒤 MISSION_COMPLETE/MISSION_FAILED를 결정한다.
"""

import math
import time

import rclpy

from .demo_state_manager import DemoStateManager
from .state_manager import StateManager


class DemoStateManager3(DemoStateManager):
    """초기 객체 처리 → 필요시 base 재탐색 → 최종 복귀 시나리오."""

    WAITING_INITIAL_TARGET = 'WAITING_INITIAL_TARGET'
    DETECTING_MISSING_TARGET = 'DETECTING_MISSING_TARGET'

    PHASE_FIRE = 'demo3_fire'
    PHASE_PERSON = 'demo3_person'
    PHASE_RETURN_FOR_RESCAN = 'demo3_return_for_rescan'
    PHASE_RESCAN_WAIT = 'demo3_rescan_wait'
    PHASE_FINALIZE = 'demo3_finalize'

    RESULT_MISSING = 'missing'

    def __init__(self):
        """Initialize demo3 mission result tracking."""
        super().__init__()
        self._reset_demo3_results()
        self.get_logger().info(
            'Demo state manager 3 ready: handle any initial object, '
            'rescan missing type at base, finalize after return'
        )

    def _reset_demo3_results(self):
        self._fire_result = None
        self._person_result = None
        self._initial_had_both = False
        self._initial_object_finished = False
        self._rescan_target_type = None

    # =========================================================
    # Mission lifecycle / result policy
    # =========================================================

    def start_mission_callback(self, request, response):
        """Reset demo3 outcomes when a new mission starts."""
        was_started = self._mission_started
        response = super().start_mission_callback(request, response)
        if not was_started and response.success:
            self._reset_demo3_results()
        return response

    def target_complete_callback(self, request, response):
        """Record an outcome and advance without early object failure."""
        # Return navigation has no active object. Intermediate and final
        # returns differ because only the latter permits final judgment.
        if (
            self.state in (
                self.RETURNING_TO_CHARGE,
                self.RETURNING_TO_BASE,
                self.RETURNING_MANUAL,
            )
            and self.active_target is None
        ):
            if request.data != self.TARGET_STATUS_SUCCESS:
                self._manual_stop = False
                self._fail_mission(f'base 복귀 실패: {request.data}')
                response.success = True
                return response

            if self.phase == self.PHASE_RETURN_FOR_RESCAN:
                self._alignment_next = self.PHASE_RESCAN_WAIT
                self.phase = self.PHASE_ALIGN_AT_BASE
                self._event_logger.info(
                    '중간 base 복귀 완료: 최초 헤딩 정렬 후 누락 객체 재탐색'
                )
            elif self.phase == self.PHASE_FINAL_RETURN:
                self._alignment_next = self.PHASE_FINALIZE
                self.phase = self.PHASE_ALIGN_AT_BASE
                self._event_logger.info(
                    '최종 base 복귀 완료: 최초 헤딩 정렬 후 결과 판정'
                )
            elif self.phase == self.PHASE_LOW_BATTERY_RETURN:
                self.phase = self.PHASE_WAITING_FOR_CHARGE
                self._event_logger.info(
                    '저전압 복귀 도착: WAITING_FOR_CHARGE (임무 완료 아님)'
                )
            elif self.phase == self.PHASE_MANUAL_RETURN:
                self._reset_demo_after_manual_stop()

            response.success = True
            return response

        if self.active_target is None:
            response.success = False
            return response

        completed_type = self.active_target['type']
        completed_phase = self.phase
        result = request.data

        # StateManager records done/unreachable/extinguished and removes the
        # target. DemoStateManager is bypassed because its override turns every
        # non-success object result into an immediate mission failure.
        response = StateManager.target_complete_callback(
            self, request, response
        )

        if completed_type == 'fire':
            self._fire_result = result
        elif completed_type == 'person':
            self._person_result = result

        self._event_logger.info(
            f'demo3 객체 처리 결과: type={completed_type}, result={result}'
        )

        if completed_phase not in (self.PHASE_FIRE, self.PHASE_PERSON):
            self._fail_mission(f'예상하지 못한 객체 완료 phase: {completed_phase}')
            return response

        if not self._initial_object_finished:
            self._initial_object_finished = True
            if self._initial_had_both:
                next_type = 'person' if completed_type == 'fire' else 'fire'
                self._set_object_phase(next_type)
                self._event_logger.info(
                    f'초기 객체 2종 확보: {next_type} 처리 계속'
                )
            else:
                self.phase = self.PHASE_RETURN_FOR_RESCAN
                self._event_logger.info(
                    f'초기에는 {completed_type}만 확보: base 복귀 후 누락 객체 재탐색'
                )
        else:
            self.phase = self.PHASE_FINAL_RETURN
            self._event_logger.info('계획된 객체 처리 종료: 최종 base 복귀')

        return response

    # =========================================================
    # State selection
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

        if self.phase == self.PHASE_WAITING_FOR_CHARGE:
            self._enter_terminal_state('WAITING_FOR_CHARGE')
            return

        if self.is_battery_low():
            self.phase = self.PHASE_LOW_BATTERY_RETURN
            self._enter_returning_when_pose_ready('low battery')
            return

        now = time.monotonic()
        if self.phase == self.PHASE_INITIAL_SWEEP:
            self._process_initial_sweep(now)
        elif self.phase == self.PHASE_CLUSTER_FIRE:
            # The inherited initial sweep enters PHASE_CLUSTER_FIRE when its
            # rotations finish. In demo3 this phase selects either object type.
            self._process_initial_targets(now)
        elif self.phase == self.PHASE_FIRE:
            self._process_object('fire')
        elif self.phase == self.PHASE_PERSON:
            self._process_object('person')
        elif self.phase == self.PHASE_RETURN_FOR_RESCAN:
            self._enter_returning_when_pose_ready('rescan missing object')
        elif self.phase == self.PHASE_ALIGN_AT_BASE:
            self._process_demo3_base_alignment()
        elif self.phase == self.PHASE_SECOND_SWEEP:
            self._process_second_sweep(now)
        elif self.phase == self.PHASE_RESCAN_WAIT:
            self._process_rescan_wait(now)
        elif self.phase == self.PHASE_FINAL_RETURN:
            self._enter_returning_when_pose_ready('demo3 objects processed')
        else:
            self._fail_mission(f'알 수 없는 demo3 phase: {self.phase}')

    # =========================================================
    # Initial selection and object dispatch
    # =========================================================

    def _spin_failed(self, message):
        """Keep detected initial targets usable when the sweep fails."""
        if self.phase == self.PHASE_INITIAL_SWEEP and (
            self._pick_pending('fire') is not None
            or self._pick_pending('person') is not None
        ):
            self._spin_pending = False
            self._spin_purpose = None
            self._spin_goal_handle = None
            self._event_logger.warn(
                f'초기 spin 실패: {message}; 인식된 객체 처리로 전환'
            )
            # Reuse normal initial selection (fire first if both exist).
            # Dispatch on the next timer tick so battery/manual-stop checks
            # retain priority over object navigation.
            self._process_initial_targets(time.monotonic())
            return

        super()._spin_failed(message)

    def _process_initial_targets(self, now):
        fire = self._pick_pending('fire')
        person = self._pick_pending('person')

        if fire is not None or person is not None:
            self._initial_had_both = fire is not None and person is not None
            # Fire suppression has priority only when both are available.
            selected_type = 'fire' if fire is not None else 'person'
            self._set_object_phase(selected_type)
            self._event_logger.info(
                '초기 객체 선택: '
                f'fire={fire is not None}, person={person is not None}, '
                f'first={selected_type}'
            )
            return

        self._enter_terminal_state(self.WAITING_INITIAL_TARGET)
        if now < self._cluster_deadline:
            return

        if self._sweep_round < self.initial_scan_max_rounds:
            self._begin_initial_sweep(now, new_round=True)
            return

        # The sweep ends at the initial heading; the robot never left base.
        self._fire_result = self.RESULT_MISSING
        self._person_result = self.RESULT_MISSING
        self._finalize_mission()

    def _pick_pending(self, target_type):
        if (
            self.active_target is not None
            and self.active_target['type'] == target_type
        ):
            return self.active_target

        candidates = [
            entry
            for entry in self.target_queue
            if entry['type'] == target_type
            and entry['status'] != 'done'
            and self._is_within_map(entry)
        ]
        return self._nearest_to_robot(candidates) if candidates else None

    def _set_object_phase(self, target_type):
        self.phase = (
            self.PHASE_FIRE if target_type == 'fire' else self.PHASE_PERSON
        )

    def _process_object(self, target_type):
        target = self._pick_pending(target_type)
        if target is not None:
            self._enter_urgent_target(target)
            return

        # A target selected from target_queue should remain there until its
        # completion callback. If it disappears, recover through the same base
        # rescan policy instead of stopping away from base.
        if not self._initial_object_finished:
            self.phase = self.PHASE_RETURN_FOR_RESCAN
        else:
            if target_type == 'fire':
                self._fire_result = self.RESULT_MISSING
            else:
                self._person_result = self.RESULT_MISSING
            self.phase = self.PHASE_FINAL_RETURN

    # =========================================================
    # Base alignment and missing-object rescan
    # =========================================================

    def _process_demo3_base_alignment(self):
        self._enter_terminal_state(self.ALIGNING_AT_BASE)
        if self._spin_pending:
            return

        if self.robot_yaw is None or self.start_yaw is None:
            self.get_logger().warn(
                '최초 헤딩 TF를 기다리는 중',
                throttle_duration_sec=2.0,
            )
            return

        yaw_error = self._normalize_angle(self.start_yaw - self.robot_yaw)
        if abs(math.degrees(yaw_error)) <= self.heading_tolerance_deg:
            if self._alignment_next == self.PHASE_RESCAN_WAIT:
                self._begin_second_sweep(time.monotonic())
            elif self._alignment_next == self.PHASE_FINALIZE:
                self._finalize_mission()
            else:
                self._fail_mission(
                    f'알 수 없는 base 정렬 후 phase: {self._alignment_next}'
                )
            return

        self._event_logger.info(
            f'최초 헤딩으로 {math.degrees(yaw_error):.1f}도 정렬'
        )
        self._send_spin(yaw_error, 'base_alignment')

    def _begin_single_fire_detection(self, now=None):
        # Reuse the inherited second-sweep rotation sequence, but wait for
        # the missing object type instead of specifically looking for fire.
        if now is None:
            now = time.monotonic()
        self._rescan_target_type = (
            'fire' if self._fire_result is None else 'person'
        )
        self.phase = self.PHASE_RESCAN_WAIT
        self._single_detection_ready_at = now + self.base_detection_dwell_sec
        self._single_detection_deadline = now + self.single_fire_timeout_sec
        self._event_logger.info(
            f'base 좌우 재스캔 완료: {self._rescan_target_type} 탐색'
        )

    def _process_rescan_wait(self, now):
        if now < self._single_detection_ready_at:
            self._enter_terminal_state(self.DETECTING_MISSING_TARGET)
            return

        target = self._pick_pending(self._rescan_target_type)
        if target is not None:
            self._set_object_phase(self._rescan_target_type)
            self._enter_urgent_target(target)
            return

        self._enter_terminal_state(self.DETECTING_MISSING_TARGET)
        if now < self._single_detection_deadline:
            return

        if self._rescan_target_type == 'fire':
            self._fire_result = self.RESULT_MISSING
        else:
            self._person_result = self.RESULT_MISSING
        self._event_logger.warn(
            f'base 재탐색 제한시간 종료: {self._rescan_target_type} 미발견'
        )
        # The robot is at base and the sweep ends at the initial heading.
        self._finalize_mission()

    def _finalize_mission(self):
        fire_ok = self._fire_result == self.TARGET_STATUS_SUCCESS
        person_ok = self._person_result == self.TARGET_STATUS_SUCCESS
        if fire_ok and person_ok:
            self.phase = self.PHASE_COMPLETE
            self._event_logger.info(
                'demo3 최종 판정: 진압 성공, person 도착 성공'
            )
        else:
            self._event_logger.error(
                'demo3 최종 판정 실패: '
                f'fire={self._fire_result}, person={self._person_result}'
            )
            self.phase = self.PHASE_FAILED
        self._refresh_state()

    def _reset_demo_after_manual_stop(self):
        super()._reset_demo_after_manual_stop()
        self._reset_demo3_results()


def main(args=None):
    """Run the demo3 state manager node."""
    rclpy.init(args=args)
    node = DemoStateManager3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
