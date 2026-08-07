#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import datetime
class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        # 기본 이미지 토픽 설정
        # self.declare_parameter('image_topic', '/ascamera/camera_publisher/rgb0/image')
        self.declare_parameter('image_topic', 'result_img')
        self.declare_parameter('output_path', f'driving_data_{datetime.datetime.now()}.avi')
        self.declare_parameter('fps', 20.0)
        
        topic = self.get_parameter('image_topic').value
        self.output_path = self.get_parameter('output_path').value
        self.fps = self.get_parameter('fps').value
        
        self.bridge = CvBridge()
        self.writer = None
        
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f"녹화 시작: {topic} 구독 중... 파일 저장 경로: {self.output_path}")

    def image_callback(self, msg):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"이미지 변환 실패: {str(e)}")
            return
        
        h, w = cv_img.shape[:2]
        if self.writer is None:
            # XVID 코덱을 사용해 AVI 파일 생성 정의
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, (w, h))
            self.get_logger().info(f"VideoWriter 초기화 완료 (해상도: {w}x{h}, FPS: {self.fps})")
            
        # 프레임 쓰기
        self.writer.write(cv_img)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info("동영상 파일 작성이 완료되어 release 되었습니다.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("사용자 인터럽트로 녹화를 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()