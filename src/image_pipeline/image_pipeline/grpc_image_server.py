#!/usr/bin/env python3

from __future__ import annotations

import threading
import time
from concurrent import futures

import cv2
import grpc
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from image_pipeline import camera_stream_pb2
from image_pipeline import camera_stream_pb2_grpc


class CameraStreamService(camera_stream_pb2_grpc.CameraStreamServicer):
    """
    최신 CameraFrame 하나만 유지하는 gRPC 스트리밍 서비스.

    네트워크가 느려져도 이전 frame을 queue에 쌓지 않습니다.
    로봇 모니터링에서는 오래된 frame보다 최신 frame이 중요하기 때문입니다.
    """

    def __init__(self, node: "GrpcImageServerNode"):
        self.node = node

        self._condition = threading.Condition()

        self._latest_frame = None
        self._latest_sequence = 0

        self._shutdown = False

    def update_frame(self, frame):
        with self._condition:
            self._latest_frame = frame
            self._latest_sequence = frame.sequence_id

            self._condition.notify_all()

    def shutdown(self):
        with self._condition:
            self._shutdown = True
            self._condition.notify_all()

    def StreamFrames(self, request, context):
        client = context.peer()

        self.node.get_logger().info(f"[gRPC] client connected: {client}")

        last_sequence = -1

        try:
            while context.is_active():

                with self._condition:

                    self._condition.wait_for(
                        lambda: (
                            self._shutdown
                            or (
                                self._latest_frame is not None
                                and self._latest_sequence != last_sequence
                            )
                        ),
                        timeout=1.0,
                    )

                    if self._shutdown:
                        break

                    if self._latest_frame is None:
                        continue

                    if self._latest_sequence == last_sequence:
                        continue

                    frame = self._latest_frame
                    last_sequence = self._latest_sequence

                yield frame

        except Exception as exc:
            self.node.get_logger().warn(f"[gRPC] client stream error: {exc}")

        finally:
            self.node.get_logger().info(f"[gRPC] client disconnected: {client}")


class GrpcImageServerNode(Node):

    def __init__(self):
        super().__init__("grpc_image_server")

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter(
            "input_topic",
            "/image_enhanced",
        )

        self.declare_parameter(
            "grpc_bind_address",
            "0.0.0.0",
        )

        self.declare_parameter(
            "grpc_port",
            50051,
        )

        self.declare_parameter(
            "jpeg_quality",
            75,
        )

        self.declare_parameter(
            "max_fps",
            10.0,
        )

        self.declare_parameter(
            "grpc_workers",
            2,
        )

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )

        self.bind_address = (
            self.get_parameter("grpc_bind_address").get_parameter_value().string_value
        )

        self.grpc_port = (
            self.get_parameter("grpc_port").get_parameter_value().integer_value
        )

        self.jpeg_quality = (
            self.get_parameter("jpeg_quality").get_parameter_value().integer_value
        )

        self.max_fps = self.get_parameter("max_fps").get_parameter_value().double_value

        grpc_workers = (
            self.get_parameter("grpc_workers").get_parameter_value().integer_value
        )

        self.jpeg_quality = max(
            1,
            min(100, self.jpeg_quality),
        )

        # ---------------------------------------------------------
        # ROS image subscriber
        # ---------------------------------------------------------

        self.bridge = CvBridge()

        self.sequence_id = 0

        self._last_encode_time = 0.0

        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        # ---------------------------------------------------------
        # gRPC server
        # ---------------------------------------------------------

        self.grpc_service = CameraStreamService(self)

        self.grpc_server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=max(1, grpc_workers)),
            options=[
                (
                    "grpc.max_send_message_length",
                    16 * 1024 * 1024,
                ),
                (
                    "grpc.max_receive_message_length",
                    16 * 1024 * 1024,
                ),
            ],
        )

        camera_stream_pb2_grpc.add_CameraStreamServicer_to_server(
            self.grpc_service,
            self.grpc_server,
        )

        server_address = f"{self.bind_address}:{self.grpc_port}"

        bound_port = self.grpc_server.add_insecure_port(server_address)

        if bound_port == 0:
            raise RuntimeError(f"gRPC port bind failed: {server_address}")

        self.grpc_server.start()

        self.get_logger().info("========================================")
        self.get_logger().info("[gRPC Camera Server]")
        self.get_logger().info(f"ROS input : {input_topic}")
        self.get_logger().info(f"gRPC      : {server_address}")
        self.get_logger().info(f"JPEG      : quality={self.jpeg_quality}")
        self.get_logger().info(f"Max FPS   : {self.max_fps}")
        self.get_logger().info("========================================")

    # -------------------------------------------------------------
    # ROS callback
    # -------------------------------------------------------------

    def image_callback(self, msg: Image):

        # ------------------------------------
        # FPS throttle
        # ------------------------------------

        now = time.monotonic()

        if self.max_fps > 0.0:

            min_interval = 1.0 / self.max_fps

            if now - self._last_encode_time < min_interval:
                return

        self._last_encode_time = now

        # ------------------------------------
        # ROS Image -> OpenCV
        # ------------------------------------

        try:
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

        except Exception as exc:

            self.get_logger().error(f"cv_bridge conversion failed: {exc}")

            return

        # ------------------------------------
        # JPEG encode
        # ------------------------------------

        success, encoded = cv2.imencode(
            ".jpg",
            image,
            [
                cv2.IMWRITE_JPEG_QUALITY,
                self.jpeg_quality,
            ],
        )

        if not success:

            self.get_logger().warn("JPEG encoding failed")

            return

        self.sequence_id += 1

        height, width = image.shape[:2]

        # ------------------------------------
        # protobuf CameraFrame
        # ------------------------------------

        frame = camera_stream_pb2.CameraFrame(
            sequence_id=self.sequence_id,
            # ★ YOLO Detection2DArray와 맞출 핵심 값
            stamp_sec=msg.header.stamp.sec,
            stamp_nanosec=msg.header.stamp.nanosec,
            frame_id=msg.header.frame_id,
            width=width,
            height=height,
            encoding="jpeg",
            jpeg_data=encoded.tobytes(),
        )

        # 최신 frame 교체
        self.grpc_service.update_frame(frame)

    # -------------------------------------------------------------

    def destroy_node(self):

        self.get_logger().info("Stopping gRPC image server...")

        self.grpc_service.shutdown()

        self.grpc_server.stop(grace=1.0)

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = GrpcImageServerNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
