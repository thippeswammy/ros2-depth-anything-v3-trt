#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageRepublisher(Node):
    def __init__(self):
        super().__init__('image_republisher_py')
        
        # Parameters for input/output topics
        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/camera/color/image_raw/compressed')
        self.declare_parameter('quality', 80) # JPEG quality

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.quality = self.get_parameter('quality').value

        self.get_logger().info(f'Subscribing to: {in_topic}')
        self.get_logger().info(f'Publishing to:  {out_topic}')

        self.sub = self.create_subscription(
            Image,
            in_topic,
            self.image_callback,
            10)
        
        self.pub = self.create_publisher(
            CompressedImage,
            out_topic,
            10)
            
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Compress to JPEG
            success, encoded_img = cv2.imencode('.jpg', cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            
            if success:
                # Create CompressedImage message
                out_msg = CompressedImage()
                out_msg.header = msg.header
                out_msg.format = 'jpeg'
                out_msg.data = encoded_img.tobytes()
                
                self.pub.publish(out_msg)
                
        except Exception as e:
            self.get_logger().error(f'Conversion error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
