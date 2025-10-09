#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraRight(Node):
    def __init__(self):
        super().__init__('video_right')

        self.publisher_ = self.create_publisher(Image, '/image_right', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video file")
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)  # 30Hz로 프레임 퍼블리시
        self.get_logger().info("video_right node started")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read camera frame")
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().debug("Frame published")

def main(args=None):
    rclpy.init(args=args)
    node = CameraRight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("video_right node stopped")
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()