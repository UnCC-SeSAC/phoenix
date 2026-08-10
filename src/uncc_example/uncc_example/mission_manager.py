import rclpy

from rclpy.executors import MultiThreadedExecutor

from .vision_detector import VisionDetector
from .state_manager import StateManager
from .mission_executor import MissionExecutor


def main(args=None):
    """
    vision_detector + state_manager + mission_executor 를 한
    프로세스에서 같이 돌린다 (RAM 절약 목적).

    yolo_detector 는 나중에 다른 사람이 작성한 코드로 교체될
    예정이라 의도적으로 여기 포함하지 않고 계속 별도 프로세스로
    둔다 — 무거운 의존성(모델 로딩 등)을 이 프로세스까지 끌고
    오지 않기 위함.

    개별 노드(state_manager, vision_detector, mission_executor)의
    ros2 run 진입점은 그대로 남아있어서, 필요하면 지금처럼 따로
    띄워서 테스트할 수도 있다.
    """

    rclpy.init(args=args)

    vision_node = VisionDetector()
    state_node = StateManager()
    mission_node = MissionExecutor()

    executor = MultiThreadedExecutor()
    executor.add_node(vision_node)
    executor.add_node(state_node)
    executor.add_node(mission_node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        vision_node.destroy_node()
        state_node.destroy_node()
        mission_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
