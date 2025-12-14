#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        # Node expects CompressedImage input
        self.image_pub = self.create_publisher(CompressedImage, 'image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback) # 30 Hz
        self.bridge = CvBridge()
        self.cnt = 0
        
        # Create a dummy image (moving gradient)
        self.width = 640
        self.height = 480

    def timer_callback(self):
        # Create dummy image
        img = np.zeros((self.height, self.width, 3), np.uint8)
        # Moving bar
        offset = (self.cnt * 5) % self.width
        cv2.rectangle(img, (offset, 0), (offset+50, self.height), (0, 255, 0), -1)
        
        # Encode as JPEG for CompressedImage
        success, encoded_img = cv2.imencode('.jpg', img)
        if not success:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_optical_frame"
        msg.format = "jpeg"
        msg.data = np.array(encoded_img).tobytes()
        
        info_msg = CameraInfo()
        info_msg.header = msg.header
        info_msg.height = self.height
        info_msg.width = self.width
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # Simple intrinsic matrix
        fx, fy = 500.0, 500.0
        cx, cy = self.width/2, self.height/2
        info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.image_pub.publish(msg)
        self.info_pub.publish(info_msg)
        self.cnt += 1
        # self.get_logger().info(f'Published frame {self.cnt}')

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
