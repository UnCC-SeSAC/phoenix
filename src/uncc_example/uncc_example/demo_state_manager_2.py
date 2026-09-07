"""확인된 화재를 처리하고 필요할 때 base에서 다시 탐색하는 상태 관리자.

시나리오:
  1. 시작점을 기준으로 -15° / +15° 초기 스캔을 수행한다.
  2. 확정된 fire 중 사람과 군집인 fire를 우선 선택한다.
  3. 군집 fire 진압 후 person을 방문한다.
  4. 저장된 다음 fire가 있으면 즉시 이동하고, 없으면 base로 복귀해 재탐색한다.
  5. 진압 후 base 재탐색에서 fire가 없을 때 임무를 완료한다.
"""

import time

import rclpy

from .demo_state_manager import DemoStateManager


class DemoStateManager2(DemoStateManager):
    """군집 fire → person → 단독 fire → 최종 base 복귀 데모."""

    def __init__(self):
        super().__init__()
        self._complete_if_no_fire = False
        self.get_logger().info(
            'Demo state manager 2 ready: known fires first, base rescan when needed'
        )

    def target_complete_callback(self, request, response):
        """최종 복귀 뒤에도 base 재탐색을 거쳐 완료 여부를 결정한다."""
        completed_phase = self.phase
        response = super().target_complete_callback(request, response)
        if (
            completed_phase == self.PHASE_FINAL_RETURN
            and response.success
            and self.phase == self.PHASE_ALIGN_AT_BASE
        ):
            self._alignment_next = self.PHASE_SECOND_SWEEP
            self._complete_if_no_fire = True
            self._event_logger.info(
                '화재 처리 후 base 복귀 완료: 정렬 및 잔여 화재 재탐색 예정')
        return response

    def _process_cluster_person(self):
        """person 방문 후 저장된 fire가 없을 때만 base로 복귀한다."""
        if any(entry['status'] != 'done' for entry in self._cluster_people):
            super()._process_cluster_person()
            return

        if self._pick_cluster_fire() is None and self._pick_single_fire() is None:
            self._complete_if_no_fire = True
            self.phase = self.PHASE_RETURN_AFTER_CLUSTER
            self._event_logger.info(
                '다음 fire 좌표 없음: base 복귀 후 재탐색')
            return

        self._complete_if_no_fire = False
        now = time.monotonic()
        self.phase = self.PHASE_SINGLE_FIRE
        self._single_detection_ready_at = now
        self._single_detection_deadline = now + self.single_fire_timeout_sec
        self._event_logger.info(
            '저장된 다음 fire 좌표 확인: base 복귀 없이 바로 진행')

    def _process_single_fire(self, now):
        """base 탐색에서 fire가 없으면 성공, 있으면 다시 처리한다."""
        if now < self._single_detection_ready_at:
            self._enter_terminal_state(self.DETECTING_SINGLE_FIRE)
            return

        cluster_fire = self._pick_cluster_fire()
        target = cluster_fire or self._pick_single_fire()
        if target is not None:
            self._complete_if_no_fire = False
            if cluster_fire is not None:
                self._cluster_people = [
                    entry for entry in (cluster_fire['cluster'] or [])
                    if entry['type'] == 'person' and entry['status'] != 'done'
                ]
                # 완료 콜백이 같은 군집 person으로 이어지게 한다.
                self.phase = self.PHASE_CLUSTER_FIRE
            self._enter_urgent_target(target)
            return

        self._enter_terminal_state(self.DETECTING_SINGLE_FIRE)
        if now < self._single_detection_deadline:
            return

        if self._complete_if_no_fire:
            self.phase = self.PHASE_COMPLETE
            self._event_logger.info(
                'base 정렬 및 재탐색 완료: 추가 fire 없음, 데모 미션 종료')
        else:
            # 이동 전 확보했던 좌표가 사라진 경우에도 보이지 않는 위치에서
            # 실패시키지 않고 base 관측 위치로 돌아간다.
            self._complete_if_no_fire = True
            self.phase = self.PHASE_RETURN_AFTER_CLUSTER
            self._event_logger.info(
                '다음 fire 좌표를 확인할 수 없음: base 복귀 후 재탐색')


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
