#!/usr/bin/env python3

from __future__ import annotations

import queue
import threading
from collections import OrderedDict

import cv2
import grpc
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection2DArray

from image_pipeline import camera_stream_pb2
from image_pipeline import camera_stream_pb2_grpc


def stamp_to_ns(sec: int, nanosec: int) -> int:
    return int(sec) * 1_000_000_000 + int(nanosec)


class FrameDetectionSynchronizer:

    def __init__(
        self,
        output_queue: queue.Queue,
        max_buffer_size: int = 30,
    ):

        self.lock = threading.Lock()

        self.frames = OrderedDict()
        self.detections = OrderedDict()

        self.output_queue = output_queue

        self.max_buffer_size = max_buffer_size

    def add_frame(
        self,
        timestamp_ns: int,
        image,
    ):

        with self.lock:

            self.frames[timestamp_ns] = image

            detection = self.detections.pop(
                timestamp_ns,
                None,
            )

            if detection is not None:

                frame = self.frames.pop(timestamp_ns)

                self._emit(
                    frame,
                    detection,
                )

            self._trim()

    def add_detection(
        self,
        timestamp_ns: int,
        detection,
    ):

        with self.lock:

            self.detections[timestamp_ns] = detection

            frame = self.frames.pop(
                timestamp_ns,
                None,
            )

            if frame is not None:

                det = self.detections.pop(timestamp_ns)

                self._emit(
                    frame,
                    det,
                )

            self._trim()

    def _emit(
        self,
        frame,
        detection,
    ):

        item = (frame, detection)

        try:
            self.output_queue.put_nowait(item)

        except queue.Full:

            # 오래된 display frame 제거
            try:
                self.output_queue.get_nowait()
            except queue.Empty:
                pass

            try:
                self.output_queue.put_nowait(item)
            except queue.Full:
                pass

    def _trim(self):

        while len(self.frames) > self.max_buffer_size:
            self.frames.popitem(last=False)

        while len(self.detections) > self.max_buffer_size:
            self.detections.popitem(last=False)


class GrpcBBoxViewer(Node):

    def __init__(self):

        super().__init__("grpc_bbox_viewer")

        self.declare_parameter(
            "grpc_server",
            "192.168.0.100:50051",
        )

        self.declare_parameter(
            "detections_topic",
            "/yolo_result",
        )

        grpc_server = (
            self.get_parameter("grpc_server").get_parameter_value().string_value
        )

        detections_topic = (
            self.get_parameter("detections_topic").get_parameter_value().string_value
        )

        self.grpc_server_address = grpc_server

        self.render_queue = queue.Queue(maxsize=1)

        self.synchronizer = FrameDetectionSynchronizer(
            output_queue=self.render_queue,
            max_buffer_size=30,
        )

        self.subscription = self.create_subscription(
            Detection2DArray,
            detections_topic,
            self.detection_callback,
            qos_profile_sensor_data,
        )

        self.stop_event = threading.Event()

        self.grpc_thread = threading.Thread(
            target=self.grpc_receiver_loop,
            daemon=True,
        )

        self.grpc_thread.start()

        self.get_logger().info("========================================")

        self.get_logger().info("[gRPC YOLO Viewer]")

        self.get_logger().info(f"gRPC : {grpc_server}")

        self.get_logger().info(f"ROS  : {detections_topic}")

        self.get_logger().info("Synchronization: exact header.stamp")

        self.get_logger().info("========================================")

    # ---------------------------------------------------------
    # Detection subscriber
    # ---------------------------------------------------------

    def detection_callback(
        self,
        msg: Detection2DArray,
    ):

        timestamp_ns = stamp_to_ns(
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
        )

        self.synchronizer.add_detection(
            timestamp_ns,
            msg,
        )

    # ---------------------------------------------------------
    # gRPC receiver
    # ---------------------------------------------------------

    def grpc_receiver_loop(self):

        while not self.stop_event.is_set():

            try:

                self.get_logger().info(
                    "Connecting to gRPC server: " f"{self.grpc_server_address}"
                )

                channel = grpc.insecure_channel(
                    self.grpc_server_address,
                    options=[
                        (
                            "grpc.max_receive_message_length",
                            16 * 1024 * 1024,
                        ),
                    ],
                )

                stub = camera_stream_pb2_grpc.CameraStreamStub(channel)

                stream = stub.StreamFrames(camera_stream_pb2.EmptyRequest())

                self.get_logger().info("gRPC image stream connected")

                for frame in stream:

                    if self.stop_event.is_set():
                        break

                    np_data = np.frombuffer(
                        frame.jpeg_data,
                        dtype=np.uint8,
                    )

                    image = cv2.imdecode(
                        np_data,
                        cv2.IMREAD_COLOR,
                    )

                    if image is None:
                        continue

                    timestamp_ns = stamp_to_ns(
                        frame.stamp_sec,
                        frame.stamp_nanosec,
                    )

                    self.synchronizer.add_frame(
                        timestamp_ns,
                        image,
                    )

            except grpc.RpcError as exc:

                if not self.stop_event.is_set():

                    self.get_logger().warn(
                        "gRPC connection failed: " f"{exc.code()} " f"{exc.details()}"
                    )

                    self.stop_event.wait(1.0)

            except Exception as exc:

                if not self.stop_event.is_set():

                    self.get_logger().error(f"gRPC receiver error: {exc}")

                    self.stop_event.wait(1.0)

    # ---------------------------------------------------------
    # bbox compatibility
    # ---------------------------------------------------------

    @staticmethod
    def get_bbox(det):

        bbox = det.bbox

        center = bbox.center

        # vision_msgs 버전 호환
        if hasattr(center, "position"):

            cx = float(center.position.x)

            cy = float(center.position.y)

        else:

            cx = float(center.x)
            cy = float(center.y)

        width = float(bbox.size_x)

        height = float(bbox.size_y)

        x1 = int(round(cx - width / 2))

        y1 = int(round(cy - height / 2))

        x2 = int(round(cx + width / 2))

        y2 = int(round(cy + height / 2))

        return x1, y1, x2, y2

    @staticmethod
    def get_label(det):

        if not det.results:
            return "unknown", 0.0

        result = det.results[0]

        hypothesis = getattr(
            result,
            "hypothesis",
            None,
        )

        if hypothesis is not None:

            class_id = str(hypothesis.class_id)

            score = float(hypothesis.score)

            return class_id, score

        # 구버전 compatibility
        class_id = str(
            getattr(
                result,
                "id",
                "unknown",
            )
        )

        score = float(
            getattr(
                result,
                "score",
                0.0,
            )
        )

        return class_id, score

    # ---------------------------------------------------------
    # Draw detections
    # ---------------------------------------------------------

    def draw_detections(
        self,
        image,
        msg: Detection2DArray,
    ):

        output = image.copy()

        height, width = output.shape[:2]

        for det in msg.detections:

            try:

                (
                    x1,
                    y1,
                    x2,
                    y2,
                ) = self.get_bbox(det)

                x1 = max(0, min(width - 1, x1))

                y1 = max(0, min(height - 1, y1))

                x2 = max(0, min(width - 1, x2))

                y2 = max(0, min(height - 1, y2))

                class_id, score = self.get_label(det)

                label = f"{class_id} " f"{score:.2f}"

                cv2.rectangle(
                    output,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    output,
                    label,
                    (
                        x1,
                        max(20, y1 - 8),
                    ),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

            except Exception as exc:

                self.get_logger().warn(f"bbox draw failed: {exc}")

        return output

    # ---------------------------------------------------------

    def render_once(self):

        try:

            image, detections = self.render_queue.get_nowait()

        except queue.Empty:
            return

        output = self.draw_detections(
            image,
            detections,
        )

        text = (
            "stamp: "
            f"{detections.header.stamp.sec}."
            f"{detections.header.stamp.nanosec:09d}"
        )

        cv2.putText(
            output,
            text,
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        cv2.imshow(
            "Phoenix YOLO gRPC Viewer",
            output,
        )

    def destroy_node(self):

        self.stop_event.set()

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = GrpcBBoxViewer()

    try:

        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.01,
            )

            node.render_once()

            key = cv2.waitKey(1)

            if key == 27:  # ESC
                break

            if key == ord("q"):
                break

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
