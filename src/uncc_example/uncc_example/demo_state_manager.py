"""고정된 데모 현장에서 탐사 없이 두 번 출동하는 상태 관리자."""

import math
import time

import rclpy

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as ActionDuration
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import Spin
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import TransformException

from .state_manager import StateManager


class DemoStateManager(StateManager):
    """초기 좌우 스캔 후 우선순위 fire와 단독 fire를 처리한다."""

    INITIAL_SWEEP = 'INITIAL_SWEEP'
    WAITING_CLUSTER_FIRE = 'WAITING_CLUSTER_FIRE'
    ALIGNING_AT_BASE = 'ALIGNING_AT_BASE'
    SECOND_SWEEP = 'SECOND_SWEEP'
    DETECTING_SINGLE_FIRE = 'DETECTING_SINGLE_FIRE'
    MISSION_COMPLETE = 'MISSION_COMPLETE'
    MISSION_FAILED = 'MISSION_FAILED'

    PHASE_INITIAL_SWEEP = 'initial_sweep'
    PHASE_CLUSTER_FIRE = 'cluster_fire'
    PHASE_CLUSTER_PERSON = 'cluster_person'
    PHASE_LOW_BATTERY_RETURN = 'low_battery_return'
    PHASE_WAITING_FOR_CHARGE = 'waiting_for_charge'
    PHASE_RETURN_AFTER_CLUSTER = 'return_after_cluster'
    PHASE_ALIGN_AT_BASE = 'align_at_base'
    PHASE_SECOND_SWEEP = 'second_sweep'
    PHASE_SINGLE_FIRE = 'single_fire'
    PHASE_FINAL_RETURN = 'final_return'
    PHASE_MANUAL_RETURN = 'manual_return'
    PHASE_COMPLETE = 'complete'
    PHASE_FAILED = 'failed'

    def __init__(self):
        super().__init__()

        self.declare_parameter('sweep_angle_deg', 15.0)
        self.declare_parameter('sweep_dwell_sec', 1.0)
        self.declare_parameter('initial_scan_max_rounds', 2)
        self.declare_parameter('cluster_detection_timeout_sec', 2.0)
        self.declare_parameter('base_detection_dwell_sec', 2.0)
        self.declare_parameter('single_fire_timeout_sec', 8.0)
        self.declare_parameter('heading_tolerance_deg', 2.0)
        self.declare_parameter('spin_time_allowance_sec', 10)

        self.sweep_angle_deg = self.get_parameter('sweep_angle_deg').value
        self.sweep_dwell_sec = self.get_parameter('sweep_dwell_sec').value
        self.initial_scan_max_rounds = self.get_parameter(
            'initial_scan_max_rounds'
        ).value
        self.cluster_detection_timeout_sec = self.get_parameter(
            'cluster_detection_timeout_sec'
        ).value
        self.base_detection_dwell_sec = self.get_parameter(
            'base_detection_dwell_sec'
        ).value
        self.single_fire_timeout_sec = self.get_parameter(
            'single_fire_timeout_sec'
        ).value
        self.heading_tolerance_deg = self.get_parameter(
            'heading_tolerance_deg'
        ).value
        self.spin_time_allowance_sec = self.get_parameter(
            'spin_time_allowance_sec'
        ).value

        self.phase = self.PHASE_INITIAL_SWEEP

        # base StateManager는 시작 위치의 x/y만 저장하므로 데모에서 yaw도
        # 같은 시점에 저장한다. 두 번째 출동 전 정면 복원에 사용한다.
        self.robot_yaw = None
        self.start_yaw = None

        self._spin_client = ActionClient(self, Spin, 'spin')
        self._spin_pending = False
        self._spin_purpose = None

        self._sweep_steps = []
        self._sweep_step_index = 0
        self._sweep_round = 0
        self._dwell_until = None
        self._cluster_deadline = None
        self._single_detection_ready_at = None
        self._single_detection_deadline = None
        self._cluster_people = []
        self._alignment_next = self.PHASE_SECOND_SWEEP
        self._return_reason = None
        self._spin_generation = 0
        self._spin_goal_handle = None

        self.get_logger().info(
            'Demo state manager ready: two-sortie scenario, no frontier exploration'
        )

    # =========================================================
    # Mission lifecycle
    # =========================================================

    def start_mission_callback(self, request, response):
        response = super().start_mission_callback(request, response)

        # 리셋(super()가 _mission_started 를 다시 False로 내림)일 때는
        # 초기화만 하고 멈춘다 — 실제로 움직이는 건 그 다음 진짜 START부터.
        if response.success and self._mission_started:
            self._begin_initial_sweep(time.monotonic(), new_round=True)

        return response

    def target_complete_callback(self, request, response):
        # All return reasons share navigation, but only successful mission
        # returns may advance the scenario. Charging is never mission success.
        if (
            self.state in (self.RETURNING_TO_CHARGE, self.RETURNING_TO_BASE,
                           self.RETURNING_MANUAL)
            and self.active_target is None
        ):
            if request.data != self.TARGET_STATUS_SUCCESS:
                self._manual_stop = False
                self._fail_mission(f'base 복귀 실패: {request.data}')
                response.success = True
                return response
            if self.phase == self.PHASE_RETURN_AFTER_CLUSTER:
                self._alignment_next = self.PHASE_SECOND_SWEEP
                self.phase = self.PHASE_ALIGN_AT_BASE
                self._event_logger.info(
                    '1차 복귀 완료: 최초 헤딩 정렬 시작'
                )
            elif self.phase == self.PHASE_FINAL_RETURN:
                self._alignment_next = self.PHASE_COMPLETE
                self.phase = self.PHASE_ALIGN_AT_BASE
                self._event_logger.info('최종 복귀 도착: 최초 헤딩 정렬 시작')
            elif self.phase == self.PHASE_LOW_BATTERY_RETURN:
                self.phase = self.PHASE_WAITING_FOR_CHARGE
                self._event_logger.info('저전압 복귀 도착: WAITING_FOR_CHARGE (임무 완료 아님)')
            elif self.phase == self.PHASE_MANUAL_RETURN:
                self._reset_demo_after_manual_stop()

            response.success = True
            return response

        completed_phase = self.phase
        completed_fire = (
            self.active_target is not None
            and self.active_target['type'] == 'fire'
        )
        if self.active_target is None:
            response.success = False
            return response
        if request.data != self.TARGET_STATUS_SUCCESS:
            self._fail_mission(f'객체 처리 실패: {request.data}')
            response.success = True
            return response
        response = super().target_complete_callback(request, response)

        if completed_fire and completed_phase == self.PHASE_CLUSTER_FIRE:
            self.phase = self.PHASE_CLUSTER_PERSON
            self._event_logger.info('1차 화재 처리 완료: 같은 군집 person 접근 시작')
        elif completed_phase == self.PHASE_CLUSTER_PERSON:
            self._process_cluster_person()
        elif completed_fire and completed_phase == self.PHASE_SINGLE_FIRE:
            self.phase = self.PHASE_FINAL_RETURN
            self._event_logger.info('단독 화재 처리 완료: 최종 base 복귀')

        return response

    # =========================================================
    # State selection
    # =========================================================

    def _refresh_state(self):
        if not self._mission_started:
            self._enter_standby()
            return

        if self._manual_stop:
            # stop_mission 이 호출되면 어느 phase에 있었든(진압 중이었어도)
            # 무조건 중단하고 base로 복귀시킨다. FIRE_DETECTED 를 벗어나는
            # 것만으로 mission_executor 가 알아서 진압을 취소한다.
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
            self._process_cluster_fire(now)
        elif self.phase == self.PHASE_CLUSTER_PERSON:
            self._process_cluster_person()
        elif self.phase == self.PHASE_RETURN_AFTER_CLUSTER:
            self._enter_returning_when_pose_ready('cluster fire/person complete')
        elif self.phase == self.PHASE_ALIGN_AT_BASE:
            self._process_base_alignment()
        elif self.phase == self.PHASE_SECOND_SWEEP:
            self._process_second_sweep(now)
        elif self.phase == self.PHASE_SINGLE_FIRE:
            self._process_single_fire(now)
        elif self.phase == self.PHASE_FINAL_RETURN:
            self._enter_returning_when_pose_ready('all demo fires complete')
        elif self.phase == self.PHASE_LOW_BATTERY_RETURN:
            self._enter_returning_when_pose_ready('low battery')
        else:
            self._enter_terminal_state(self.MISSION_FAILED)

    # =========================================================
    # Initial -15 / +15 degree sweep
    # =========================================================

    def _begin_initial_sweep(self, now, new_round):
        angle = math.radians(self.sweep_angle_deg)
        # Spin 목표는 상대각이다: 0 -> -15 -> +15 -> 0.
        self._sweep_steps = [-angle, 2.0 * angle, -angle]
        self._sweep_step_index = 0
        self._dwell_until = now + self.sweep_dwell_sec
        self._cluster_deadline = None
        self.phase = self.PHASE_INITIAL_SWEEP

        if new_round:
            self._sweep_round += 1

        self._event_logger.info(
            f'초기 좌우 스캔 {self._sweep_round}/'
            f'{self.initial_scan_max_rounds} 시작'
        )

    def _process_initial_sweep(self, now):
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

        self.phase = self.PHASE_CLUSTER_FIRE
        self._cluster_deadline = now + self.cluster_detection_timeout_sec
        self._event_logger.info('초기 좌우 스캔 완료: 군집 화재 선택')

    def _process_cluster_fire(self, now):
        """군집 fire를 우선하고, 없으면 단독 fire를 첫 목표로 고른다."""
        cluster_fire = self._pick_cluster_fire()
        single_fire = self._pick_single_fire()
        target = cluster_fire or single_fire

        if target is not None:
            if self.active_target is None:
                self._cluster_people = [entry for entry in (target['cluster'] or [])
                                        if entry['type'] == 'person'
                                        and entry['status'] != 'done']
            self._enter_urgent_target(target)
            return

        self._enter_terminal_state(self.WAITING_CLUSTER_FIRE)

        if now < self._cluster_deadline:
            return

        if self._sweep_round < self.initial_scan_max_rounds:
            self._begin_initial_sweep(now, new_round=True)
            return

        self._fail_mission('초기 좌우 스캔에서 처리할 fire를 찾지 못함')

    # =========================================================
    # Base heading alignment and second fire
    # =========================================================

    def _process_base_alignment(self):
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
            if self._alignment_next == self.PHASE_COMPLETE:
                self.phase = self.PHASE_COMPLETE
                self._event_logger.info('최종 복귀 정렬 완료: 데모 미션 종료')
            else:
                self._begin_second_sweep(time.monotonic())
            return

        self._event_logger.info(
            f'최초 헤딩으로 {math.degrees(yaw_error):.1f}도 정렬'
        )
        self._send_spin(yaw_error, 'base_alignment')

    def _begin_second_sweep(self, now):
        """1차 복귀 후 최초 헤딩을 기준으로 좌우를 다시 관측한다."""

        angle = math.radians(self.sweep_angle_deg)
        self._sweep_steps = [-angle, 2.0 * angle, -angle]
        self._sweep_step_index = 0
        self._dwell_until = now + self.sweep_dwell_sec
        self.phase = self.PHASE_SECOND_SWEEP
        self._event_logger.info(
            '1차 복귀 후 최초 헤딩 기준 좌우 재스캔 시작 '
            '(-15° → +15° → 정면)'
        )

    def _process_second_sweep(self, now):
        self._enter_terminal_state(self.SECOND_SWEEP)

        if self._spin_pending:
            return

        if self._dwell_until is not None and now < self._dwell_until:
            return

        self._dwell_until = None

        if self._sweep_step_index < len(self._sweep_steps):
            angle = self._sweep_steps[self._sweep_step_index]
            self._send_spin(angle, 'second_sweep')
            return

        # 마지막 -15° 회전이 끝나면 이미 최초 헤딩으로 돌아와 있다.
        # 마지막 정면 관측 시간을 조금 더 주고 단독 fire를 선택한다.
        self._begin_single_fire_detection(now)

    def _begin_single_fire_detection(self, now=None):
        if now is None:
            now = time.monotonic()
        self.phase = self.PHASE_SINGLE_FIRE
        self._single_detection_ready_at = now + self.base_detection_dwell_sec
        self._single_detection_deadline = now + self.single_fire_timeout_sec
        self._event_logger.info(
            f'좌우 재스캔 완료: 정면을 {self.base_detection_dwell_sec:.1f}초 '
            '추가 관측한 뒤 단독 화재 선택'
        )

    def _process_single_fire(self, now):
        if now < self._single_detection_ready_at:
            self._enter_terminal_state(self.DETECTING_SINGLE_FIRE)
            return

        target = self._pick_single_fire()

        if target is not None:
            self._enter_urgent_target(target)
            return

        self._enter_terminal_state(self.DETECTING_SINGLE_FIRE)

        if now >= self._single_detection_deadline:
            self._fail_mission(
                '단독 화재를 제한시간 내 찾지 못함'
            )

    # =========================================================
    # Target selection
    # =========================================================

    def _process_cluster_person(self):
        candidates = [entry for entry in self._cluster_people
                      if entry['status'] != 'done']
        if candidates:
            target = self.active_target or self._nearest_to_robot(candidates)
            if not self._is_within_map(target):
                self._fail_mission('군집 person이 지도 범위 밖에 있음')
                return
            self._enter_urgent_target(target)
        else:
            self.phase = self.PHASE_RETURN_AFTER_CLUSTER
            self._event_logger.info('1차 fire/person 처리 완료: 1차 base 복귀')

    def _pick_cluster_fire(self):
        if self.active_target is not None:
            return self.active_target

        candidates = [
            entry for entry in self.target_queue
            if entry['type'] == 'fire'
            and self._is_within_map(entry)
            and self._has_person_in_cluster(entry)
        ]
        return self._nearest_to_robot(candidates) if candidates else None

    def _pick_single_fire(self):
        if self.active_target is not None:
            return self.active_target

        candidates = [
            entry for entry in self.target_queue
            if entry['type'] == 'fire'
            and self._is_within_map(entry)
            and not self._has_person_in_cluster(entry)
        ]
        return self._nearest_to_robot(candidates) if candidates else None

    @staticmethod
    def _has_person_in_cluster(entry):
        return (
            entry['cluster'] is not None
            and any(
                member['type'] == 'person'
                for member in entry['cluster']
            )
        )

    # =========================================================
    # Spin action
    # =========================================================

    def _send_spin(self, relative_yaw, purpose):
        if not self._spin_client.server_is_ready():
            self.get_logger().warn(
                'spin 액션 서버가 아직 준비되지 않음',
                throttle_duration_sec=2.0,
            )
            return

        goal = Spin.Goal()
        goal.target_yaw = relative_yaw
        goal.time_allowance = ActionDuration(
            sec=int(self.spin_time_allowance_sec)
        )

        self._spin_pending = True
        self._spin_purpose = purpose
        generation = self._spin_generation
        future = self._spin_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self._spin_goal_response(result, generation))

    def _spin_goal_response(self, future, generation):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._spin_failed(f'spin goal 전송 실패: {exc}')
            return

        if generation != self._spin_generation:
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self._spin_failed('spin goal이 거부됨')
            return

        self._spin_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._spin_goal_result(result, generation))

    def _spin_goal_result(self, future, generation):
        if generation != self._spin_generation:
            return
        self._spin_goal_handle = None
        try:
            status = future.result().status
        except Exception as exc:
            self._spin_failed(f'spin 결과 수신 실패: {exc}')
            return

        purpose = self._spin_purpose
        self._spin_pending = False
        self._spin_purpose = None

        if status != GoalStatus.STATUS_SUCCEEDED:
            self._spin_failed(f'spin 실패(status={status})')
            return

        if purpose in ('initial_sweep', 'second_sweep'):
            self._sweep_step_index += 1
            self._dwell_until = time.monotonic() + self.sweep_dwell_sec
        elif purpose == 'base_alignment':
            # 다음 timer에서 실제 TF 오차를 다시 확인한다.
            self._event_logger.info('base 헤딩 정렬 spin 완료')

    def _spin_failed(self, message):
        self._spin_pending = False
        self._spin_purpose = None
        self._fail_mission(message)

    # =========================================================
    # Pose / return helpers
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

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.robot_x = translation.x
        self.robot_y = translation.y
        self.robot_yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )

        if self.start_x is None and self.start_y is None:
            self.start_x = self.robot_x
            self.start_y = self.robot_y
            self.start_yaw = self.robot_yaw

    def _enter_returning_when_pose_ready(self, reason):
        if (
            self.start_x is None
            or self.start_y is None
            or self.start_yaw is None
        ):
            self.get_logger().warn(
                f'시작 pose TF를 기다리는 중 ({reason})',
                throttle_duration_sec=2.0,
            )
            return

        return_state = (self.RETURNING_TO_CHARGE if reason == 'low battery'
                        else self.RETURNING_MANUAL if reason == 'manual stop'
                        else self.RETURNING_TO_BASE)
        if self.state != return_state or self._return_reason != reason:
            self._spin_generation += 1
            if self._spin_goal_handle is not None:
                self._spin_goal_handle.cancel_goal_async()
                self._spin_goal_handle = None
            self._spin_pending = False
            self._spin_purpose = None
            self._event_logger.info(
                f'Base return: reason={reason}, phase={self.phase}, '
                f'battery_raw={self.latest_battery}, threshold={self.low_battery_threshold}')
        self._return_reason = reason
        self.state = return_state
        self.active_target = None

        start_pose = PoseStamped()
        start_pose.header.frame_id = self.map_frame
        start_pose.pose.position.x = self.start_x
        start_pose.pose.position.y = self.start_y
        start_pose.pose.orientation.z = math.sin(self.start_yaw / 2.0)
        start_pose.pose.orientation.w = math.cos(self.start_yaw / 2.0)
        self._publish(start_pose)

    def _enter_terminal_state(self, state):
        self.state = state
        self.active_target = None
        self._publish(None)

    def _reset_mission_records(self):
        super()._reset_mission_records()

        # base 는 start_x/y 만 안다 — slam 리셋으로 map 좌표계가 바뀌면
        # 데모가 쓰는 start_yaw 도 같이 버려야 다음 TF 에서 셋이 함께
        # 새 좌표계 기준으로 다시 잡힌다 (_update_robot_pose 참고).
        self.start_yaw = None

        self.phase = self.PHASE_INITIAL_SWEEP
        self._sweep_round = 0
        self._sweep_steps = []
        self._sweep_step_index = 0
        self._dwell_until = None
        self._cluster_deadline = None
        self._single_detection_ready_at = None
        self._single_detection_deadline = None
        self._spin_pending = False
        self._spin_purpose = None
        self._cluster_people = []
        self._return_reason = None

    def _reset_demo_after_manual_stop(self):
        super()._reset_after_manual_stop()
        self._event_logger.info('수동 정지: 홈 도착, 데모 시나리오를 처음 상태로 초기화')

    def _fail_mission(self, reason):
        if self.phase != self.PHASE_FAILED:
            self.get_logger().error(f'데모 미션 실패: {reason}')
        self.phase = self.PHASE_FAILED
        self._enter_terminal_state(self.MISSION_FAILED)

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = DemoStateManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
