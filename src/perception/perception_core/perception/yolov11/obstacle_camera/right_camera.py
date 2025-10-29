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

        # Set camera format to MJPG and resolution to 640x480
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
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

# import rclpy
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# from rclpy.node import Node
# from rclpy.qos import QoSProfile

# class CameraRightNode(Node):
#     def __init__(self):
#         super().__init__('video_publisher')
        
#         self.bridge = CvBridge()
        
#         # Set the desired resolution
#         self.width, self.height = 640, 480
        
#         # Set the camera's resolution
#         self.cap = cv2.VideoCapture(0)
#         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
#         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

#         if not self.cap.isOpened():
#             self.get_logger().error('Failed to open video capture')
#             return
        
#         # QoS Profile for image topic
#         qos_profile = QoSProfile(depth=10)
        
#         # ROS 2 Publisher
#         self.cam_exam = self.create_publisher(Image, '/image_right', qos_profile)

#         # Timer to handle periodic publishing at 30Hz
#         self.timer = self.create_timer(1/30, self.publish_image)

#     def publish_image(self):
#         ret, frame = self.cap.read()
#         if not ret:
#             self.get_logger().error('Failed to capture frame')
#             return
        
#         # Convert to ROS Image message
#         img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
#         img_msg.header.stamp = self.get_clock().now().to_msg()
        
#         # Publish image message
#         self.cam_exam.publish(img_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraRightNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.cap.release()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()