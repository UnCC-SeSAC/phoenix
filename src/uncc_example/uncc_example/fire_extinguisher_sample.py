import rclpy

from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class FireExtinguisher(Node):
    """
    불 끄는 동작을 담당하는 노드.

    TODO: 실제 진화 동작(팔 제어, 소화기 분사 등)으로 교체 예정.
    지금은 mission_executor 파이프라인 테스트용으로 로그만 남기고
    바로 성공을 report 하는 더미 구현이다.

    ~/extinguish 서비스는 "시작해라" 트리거일 뿐이고, 실제
    성공/실패 결과는 /fire_extinguisher/result 토픽으로 비동기로
    알린다 — 진화 동작이 오래 걸릴 수 있어서, 결과를 서비스 응답이
    아니라 별도 토픽으로 받기로 했기 때문.
    """

    def __init__(self):
        super().__init__('fire_extinguisher')

        self.result_pub = self.create_publisher(
            Bool,
            '~/result',
            10,
        )

        self.create_service(
            Trigger,
            '~/extinguish',
            self.extinguish_callback,
        )

    def extinguish_callback(self, request, response):

        self.get_logger().info('불 끄기 동작')

        # TODO: 실제로는 여기서 동작을 시작만 하고, 끝났을 때
        # result_pub.publish() 를 나중에 호출해야 한다. 지금은
        # 더미라 바로 성공으로 report.
        result = Bool()
        result.data = True
        self.result_pub.publish(result)

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
