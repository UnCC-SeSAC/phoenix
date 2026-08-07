import rclpy

from rclpy.node import Node

from std_srvs.srv import Trigger


class FireExtinguisher(Node):
    """
    불 끄는 동작을 담당하는 노드.

    TODO: 실제 진화 동작(팔 제어, 소화기 분사 등)으로 교체 예정.
    지금은 mission_executor 파이프라인 테스트용으로 로그만 남기고
    바로 성공을 반환하는 더미 구현이다.
    """

    def __init__(self):
        super().__init__('fire_extinguisher')

        self.create_service(
            Trigger,
            '~/extinguish',
            self.extinguish_callback,
        )

    def extinguish_callback(self, request, response):

        self.get_logger().info('불 끄기 동작')

        response.success = True
        return response


def main(args=None):

    rclpy.init(args=args)

    node = FireExtinguisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
