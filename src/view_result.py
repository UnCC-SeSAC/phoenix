#!/usr/bin/env python3
# encoding: utf-8
# self_driving 검출 결과 이미지를 별도 OpenCV 창으로 가볍게 보는 뷰어
#
# rqt_image_view 가 무겁고 프레임을 버퍼링해서 끊기는 문제를 피하기 위해,
# BEST_EFFORT QoS + 큐 깊이 1 로 구독하여 항상 최신 프레임만 표시한다.
#
# 사용법:
#   python3 view_result.py                         # 기본 토픽: /self_driving/image_result
#   python3 view_result.py /yolov5_ros2/result_img # 다른 토픽 지정
#   python3 view_result.py /self_driving/image_result rgb8  # 인코딩 지정(기본 bgr8)
#
# 종료: 창에서 q 또는 ESC

import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class ResultViewer(Node):
    def __init__(self, topic, encoding):
        super().__init__('result_viewer')
        self.bridge = CvBridge()
        self.encoding = encoding
        self.latest = None  # 최신 프레임 보관 (콜백에서 갱신, 메인에서 표시)

        # 최신 프레임만 받도록 BEST_EFFORT + depth 1
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(Image, topic, self.image_callback, qos)
        self.get_logger().info(f'subscribing: {topic} (encoding={encoding})')

    def image_callback(self, msg):
        # 받은 이미지를 BGR 로 변환해 보관만 한다(표시는 메인 루프에서)
        try:
            img = self.bridge.imgmsg_to_cv2(msg, self.encoding)
            if self.encoding == 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            self.latest = img
        except Exception as e:
            self.get_logger().warn(f'convert fail: {e}')


def main():
    # 인자 파싱: [토픽] [인코딩]
    topic = sys.argv[1] if len(sys.argv) > 1 else '/self_driving/image_result'
    encoding = sys.argv[2] if len(sys.argv) > 2 else 'bgr8'

    rclpy.init()
    node = ResultViewer(topic, encoding)

    win = 'result'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while rclpy.ok():
            # 콜백 1회 처리(논블로킹). 최신 프레임을 받아 latest 갱신
            rclpy.spin_once(node, timeout_sec=0.005)
            if node.latest is not None:
                cv2.imshow(win, node.latest)
            # waitKey 가 GUI 이벤트 처리 + 프레임 페이싱 담당
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
