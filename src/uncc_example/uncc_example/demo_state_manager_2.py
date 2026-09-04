"""두 화재를 한 번의 초기 스캔 후 연속 처리하는 데모 상태 관리자.

시나리오:
  1. 시작점을 기준으로 -15° / +15° 초기 스캔을 수행한다.
  2. 사람과 군집인 fire 및 단독 fire가 모두 확정될 때까지 기다린다.
  3. 군집 fire를 진압하고, base로 돌아가지 않은 채 단독 fire를 진압한다.
  4. 두 번째 진압이 끝난 뒤에만 시작점으로 최종 복귀한다.
"""

import time

import rclpy

from .demo_state_manager import DemoStateManager
from .state_manager import StateManager


class DemoStateManager2(DemoStateManager):
    """군집 fire → 단독 fire → 최종 base 복귀 데모."""

    def __init__(self):
        super().__init__()
        self.get_logger().info(
            'Demo state manager 2 ready: cluster fire -> single fire -> base'
        )

    def target_complete_callback(self, request, response):
        """첫 진압 뒤 중간 복귀 대신 큐의 단독 fire로 바로 진행한다."""
        if (
            self.state == self.RETURNING_TO_CHARGE
            and self.active_target is None
        ):
            return super().target_complete_callback(request, response)

        completed_phase = self.phase
        completed_fire = (
            self.active_target is not None
            and self.active_target['type'] == 'fire'
        )
        response = StateManager.target_complete_callback(self, request, response)

        if completed_fire and completed_phase == self.PHASE_CLUSTER_FIRE:
            # 초기 스캔에서 이미 확인한 단독 fire를 바로 선택한다. 혹시
            # 객체 확정이 지연됐을 때만 single_fire_timeout 동안 기다린다.
            now = time.monotonic()
            self.phase = self.PHASE_SINGLE_FIRE
            self._single_detection_ready_at = now
            self._single_detection_deadline = now + self.single_fire_timeout_sec
            self._event_logger.info(
                '군집 화재 처리 완료: base 복귀 없이 단독 화재로 진행'
            )
        elif completed_fire and completed_phase == self.PHASE_SINGLE_FIRE:
            self.phase = self.PHASE_FINAL_RETURN
            self._event_logger.info('단독 화재 처리 완료: 최종 base 복귀')

        return response

    def _process_cluster_fire(self, now):
        """두 목표가 모두 초기 스캔에서 확보된 뒤에만 첫 출동한다."""
        cluster_fire = self._pick_cluster_fire()
        single_fire = self._pick_single_fire()

        if cluster_fire is not None and single_fire is not None:
            self._enter_urgent_target(cluster_fire)
            return

        self._enter_terminal_state(self.WAITING_CLUSTER_FIRE)

        if now < self._cluster_deadline:
            return

        if self._sweep_round < self.initial_scan_max_rounds:
            self._begin_initial_sweep(now, new_round=True)
            return

        self._fail_mission(
            '초기 좌우 스캔에서 군집 fire와 단독 fire를 모두 찾지 못함'
        )


def main(args=None):
    rclpy.init(args=args)
    node = DemoStateManager2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
