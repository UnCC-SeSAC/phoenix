#!/usr/bin/env python3
"""Phoenix 자율주행 하드웨어 controller.

frontier/fire-suppression 하드웨어 테스트에서 기존
``ros_robot_controller_node``를 대체하는 경량 노드다. 자율주행에 필요한
데이터와 명령 경로만 유지한다.

* board IMU -> ``/ros_robot_controller/imu_raw`` -> EKF
* board battery -> ``/ros_robot_controller/battery`` -> state manager
* Nav2 motor command -> ``/ros_robot_controller/set_motor`` -> board

화재진압 노드는 라즈베리파이 GPIO13(펌프)과 GPIO18(분사 서보)을 직접
제어한다. 이 노드는 PWM/bus-servo 명령을 설정하거나 발행하지 않으므로,
해당 GPIO 자원을 점유하거나 방해하지 않는다.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16
from std_srvs.srv import Trigger

from ros_robot_controller.ros_robot_controller_sdk import Board
from ros_robot_controller_msgs.msg import MotorsState


class RosPhoenixController(Node):
    """Phoenix 자율주행용 최소 보드 인터페이스."""

    GRAVITY = 9.80665

    def __init__(self):
        # 기존 node name을 유지해 caller의 remapping 없이도 상대 토픽이
        # /ros_robot_controller/... 경로를 계속 사용하게 한다.
        super().__init__("ros_robot_controller")

        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("imu_period_s", 0.02)
        self.declare_parameter("battery_period_s", 0.5)

        self._imu_frame = self.get_parameter("imu_frame").value
        imu_period = float(self.get_parameter("imu_period_s").value)
        battery_period = float(self.get_parameter("battery_period_s").value)
        if imu_period <= 0.0 or battery_period <= 0.0:
            raise ValueError("imu_period_s and battery_period_s must be positive")

        self._board = Board()
        self._board.enable_reception()
        # Nav2가 아직 명령을 발행하지 않은 상태에서 노드가 재시작해도
        # 안전하게 정지한 상태로 시작한다.
        self._board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

        self._imu_pub = self.create_publisher(Imu, "~/imu_raw", 1)
        self._battery_pub = self.create_publisher(UInt16, "~/battery", 1)
        self.create_subscription(
            MotorsState, "~/set_motor", self._set_motor_state, 10
        )
        self.create_service(Trigger, "~/init_finish", self._get_node_state)

        # 저주기 battery 읽기가 IMU/EKF 발행 경로를 지연시키지 않도록
        # timer를 분리한다.
        self.create_timer(imu_period, self._publish_imu)
        self.create_timer(battery_period, self._publish_battery)
        self.get_logger().info(
            f"Phoenix controller ready: IMU {1.0 / imu_period:.0f} Hz, "
            f"battery {1.0 / battery_period:.1f} Hz, motor input enabled"
        )

    def _get_node_state(self, _request, response):
        response.success = True
        return response

    def _set_motor_state(self, msg: MotorsState):
        commands = []
        for motor in msg.data:
            commands.append([motor.id, motor.rps])
        if commands:
            self._board.set_motor_speed(commands)

    def _publish_battery(self):
        value = self._board.get_battery()
        if value is None:
            return

        msg = UInt16()
        msg.data = value
        self._battery_pub.publish(msg)

    def _publish_imu(self):
        data = self._board.get_imu()
        if data is None:
            return

        ax, ay, az, gx, gy, gz = data
        msg = Imu()
        msg.header.frame_id = self._imu_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        # 기존 메시지 형식을 유지한다. 현재 EKF 설정은 이 IMU 토픽의 yaw와
        # yaw rate를 융합한다.
        msg.orientation.w = 0.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)
        msg.linear_acceleration.x = ax * self.GRAVITY
        msg.linear_acceleration.y = ay * self.GRAVITY
        msg.linear_acceleration.z = az * self.GRAVITY

        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        msg.linear_acceleration_covariance = [
            0.0004, 0.0, 0.0,
            0.0, 0.0004, 0.0,
            0.0, 0.0, 0.004,
        ]
        self._imu_pub.publish(msg)

    def stop_motors(self):
        """정상 종료 시 가능한 범위에서 모터를 정지한다."""
        try:
            self._board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])
        except Exception as exc:  # 하드웨어 연결이 이미 끊겼을 수 있다.
            self.get_logger().warn(f"Unable to stop motors during shutdown: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = RosPhoenixController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
