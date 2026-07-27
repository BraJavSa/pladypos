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
        self.declare_parameter('mode', 'depth') # 'depth' or 'ir'
        self.declare_parameter('max_depth', 12.0) # For depth mode
        self.declare_parameter('ir_scale', 0.02) # For ir mode (0.02 is legacy default)
        self.declare_parameter('width', -1) # Output width (-1 means keep original)
        self.declare_parameter('height', -1) # Output height (-1 means keep original)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.max_depth_m = self.get_parameter('max_depth').get_parameter_value().double_value
        self.ir_scale = self.get_parameter('ir_scale').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value

        self.bridge = CvBridge()
        
        # Using queue size of 1 to prevent latency accumulation
        self.publisher = self.create_publisher(Image, self.output_topic, 1)
        
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.listener_callback,
            1
        )
        self.get_logger().info(
            f"Optimized {self.mode.upper()} conversion: {self.input_topic} -> {self.output_topic}. "
            f"Resize target: {self.width}x{self.height}"
        )

    def listener_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV without copying data
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            if self.mode == 'ir':
                # Convert 16-bit IR to 8-bit using the scale factor
                # cv2.convertScaleAbs does: dst = clip(abs(img * alpha + beta), 0, 255)
                mono8_img = cv2.convertScaleAbs(img, alpha=self.ir_scale, beta=0.0)
            else:
                # Standard DEPTH mode
                max_depth_mm = self.max_depth_m * 1000.0
                valid_mask = img > 0
                alpha = -255.0 / max_depth_mm
                beta = 255.0

                mono8_img = cv2.convertScaleAbs(img, alpha=alpha, beta=beta)
                mono8_img[~valid_mask] = 0

            # Resize if requested
            if self.width > 0 and self.height > 0:
                mono8_img = cv2.resize(mono8_img, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

            # Publish the 8-bit image
            out_msg = self.bridge.cv2_to_imgmsg(mono8_img, encoding="mono8")
            out_msg.header = msg.header
            self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

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
