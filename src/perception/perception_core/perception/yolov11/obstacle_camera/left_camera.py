#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraLeft(Node):
    def __init__(self):
        super().__init__('video_left')
        
        self.publisher_ = self.create_publisher(Image, '/image_left', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video file")
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)  # 30Hz로 프레임 퍼블리시
        self.get_logger().info("video_left node started")

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
    node = CameraLeft()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("video_left node stopped")
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()