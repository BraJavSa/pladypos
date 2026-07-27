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
        self.declare_parameter('input_topic', '/usv5/camera/depth_raw')
        self.declare_parameter('output_topic', '/usv5/camera/depth')
        self.declare_parameter('max_depth', 12.0) # Maximum distance in meters (Kinect limit is 12m)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.max_depth_m = self.get_parameter('max_depth').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, self.output_topic, 10)
        
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.listener_callback,
            10
        )
        self.get_logger().info(
            f"Optimized conversion: {self.input_topic} -> {self.output_topic}. Max depth: {self.max_depth_m}m"
        )

    def listener_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV without copying data
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Determine max depth in millimeters (standard for 16UC1)
            max_depth_mm = self.max_depth_m * 1000.0

            # Create a mask for valid depth pixels (greater than 0)
            valid_mask = depth_img > 0

            # Calculate scale factor for mapping:
            # x = 0    => y = 255
            # x = max  => y = 0
            # Formula: y = - (255 / max_depth_mm) * x + 255
            alpha = -255.0 / max_depth_mm
            beta = 255.0

            # cv2.convertScaleAbs applies the scale, offset, and absolute cast to 8-bit in C++ (extremely fast!)
            mono8_img = cv2.convertScaleAbs(depth_img, alpha=alpha, beta=beta)

            # Set invalid pixels (depth == 0) to 0 (black)
            mono8_img[~valid_mask] = 0

            # Publish the 8-bit image
            out_msg = self.bridge.cv2_to_imgmsg(mono8_img, encoding="mono8")
            out_msg.header = msg.header
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error converting depth: {e}")

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
