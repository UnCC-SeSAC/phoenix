"""두 화재를 한 번의 초기 스캔 후 연속 처리하는 데모 상태 관리자.

시나리오:
  1. 시작점을 기준으로 -15° / +15° 초기 스캔을 수행한다.
  2. 확정된 fire 중 사람과 군집인 fire를 우선 선택한다.
  3. 군집 fire 진압 후 person을 방문하고, 중간 복귀 없이 단독 fire를 진압한다.
  4. 두 번째 진압이 끝난 뒤에만 시작점으로 최종 복귀한다.
"""

import time

import rclpy

from .demo_state_manager import DemoStateManager


class DemoStateManager2(DemoStateManager):
    """군집 fire → person → 단독 fire → 최종 base 복귀 데모."""

    def __init__(self):
        super().__init__()
        self.get_logger().info(
            'Demo state manager 2 ready: cluster fire -> person -> single fire -> base'
        )

    def _process_cluster_person(self):
        """군집 person 방문 후 중간 base 복귀 없이 단독 fire로 진행한다."""
        if any(entry['status'] != 'done' for entry in self._cluster_people):
            super()._process_cluster_person()
            return
        now = time.monotonic()
        self.phase = self.PHASE_SINGLE_FIRE
        self._single_detection_ready_at = now
        self._single_detection_deadline = now + self.single_fire_timeout_sec
        self._event_logger.info(
            '군집 fire/person 처리 완료: base 복귀 없이 단독 fire로 진행')


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
