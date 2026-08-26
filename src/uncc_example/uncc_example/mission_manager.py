import rclpy

from rclpy.executors import SingleThreadedExecutor

from .vision_detector import VisionDetector
from .state_manager import StateManager
from .mission_executor import MissionExecutor


def main(args=None):
    """
    vision_detector + state_manager + mission_executor 를 한
    프로세스에서 같이 돌린다 (RAM 절약 목적).

    비전 파이프라인(image_pipeline)은 의도적으로 여기 포함하지
    않고 계속 별도 프로세스로 둔다 — 무거운 의존성(모델 로딩 등)을
    이 프로세스까지 끌고 오지 않기 위함.

    개별 노드(state_manager, vision_detector, mission_executor)의
    ros2 run 진입점은 그대로 남아있어서, 필요하면 지금처럼 따로
    띄워서 테스트할 수도 있다.

    세 노드 콜백이 전부 기본 MutuallyExclusiveCallbackGroup이라
    실질적인 병렬 이득이 없는데, MultiThreadedExecutor는 CPU 코어
    수만큼 스레드를 띄워서 라즈베리파이에서 SLAM/nav2와 코어를
    불필요하게 더 경합했음 -> SingleThreadedExecutor로 변경.

    vision_detector 는 state_manager 가 만든 tf_buffer 를 공유받아
    쓴다 - 같은 프로세스 안에서 /tf 를 두 번 구독/파싱하지 않기 위함.
    """

    rclpy.init(args=args)

    state_node = StateManager()
    vision_node = VisionDetector(tf_buffer=state_node.tf_buffer)
    mission_node = MissionExecutor()

    executor = SingleThreadedExecutor()
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
