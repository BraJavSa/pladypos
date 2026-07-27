#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthToMono8(Node):
    def __init__(self):
        super().__init__('depth_to_mono8')

        # Declare parameters
        self.declare_parameter('input_topic', '/usv5/camera/depth')
        self.declare_parameter('output_topic', '/usv5/camera/depth_mono')
        self.declare_parameter('max_depth', 5.0) # Maximum distance in meters to map to black

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.max_depth_m = self.get_parameter('max_depth').get_parameter_value().double_value

        self.bridge = CvBridge()

        # Publisher for the 8-bit displayable depth image
        self.publisher = self.create_publisher(Image, self.output_topic, 10)

        # Subscriber to the raw 16-bit depth image
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.listener_callback,
            10
        )
        self.get_logger().info(
            f"Converting {self.input_topic} (16-bit) to {self.output_topic} (8-bit grayscale). "
            f"Max depth set to {self.max_depth_m} meters."
        )

    def listener_callback(self, msg):
        try:
            # 1. Convert ROS Image to OpenCV.
            # Kinect v2 depth is typically 16UC1 (16-bit unsigned depth in mm)
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # 2. Convert to float32 for normalized scaling
            depth_img_float = depth_img.astype(np.float32)

            # Convert mm to meters if encoding is 16UC1
            if msg.encoding == "16UC1":
                depth_img_float /= 1000.0

            # 3. Scale and invert: Closer = Whiter (255), Farther/Invalid = Black (0)
            # Map 0 meters to 255, and max_depth_m to 0.
            # Equation: y = 255 * (1 - x / max_depth)
            mono8_img = 255.0 * (1.0 - (depth_img_float / self.max_depth_m))
            
            # Clip values between 0 and 255
            mono8_img = np.clip(mono8_img, 0, 255).astype(np.uint8)

            # Handle invalid depth values (which are 0 in Kinect depth data)
            # A depth of exactly 0 mm usually means invalid reading / out of range.
            # We want invalid readings to be black (0) instead of bright white (255).
            mono8_img[depth_img == 0] = 0

            # 4. Convert back to ROS Image and publish
            out_msg = self.bridge.cv2_to_imgmsg(mono8_img, encoding="mono8")
            out_msg.header = msg.header
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthToMono8()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
