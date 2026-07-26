#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class KinectToV4L2(Node):
    def __init__(self):
        super().__init__('kinect_to_v4l2')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/kinect2/sd/image_color_rect')
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('fps', 30.0)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.video_device = self.get_parameter('video_device').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.writer = None

        # Verify device existence
        if not os.path.exists(self.video_device):
            self.get_logger().error(
                f"Device '{self.video_device}' does not exist! "
                "Did you load the v4l2loopback module? "
                "Run: sudo modprobe v4l2loopback devices=1 video_nr=0 card_label=\"KinectV2\" exclusive_caps=1"
            )
            # We don't exit immediately to allow dynamic loading
        
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Subscribed to {self.input_topic}. Ready to write to {self.video_device}...")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w, c = cv_image.shape

            if self.writer is None:
                # Open the loopback device using the V4L2 backend
                self.writer = cv2.VideoWriter(
                    self.video_device,
                    cv2.CAP_V4L2,
                    cv2.VideoWriter_fourcc(*'YUYV'),
                    self.fps,
                    (w, h)
                )
                if not self.writer.isOpened():
                    self.get_logger().error(f"Failed to open {self.video_device} for writing! Check permissions (sudo chmod 666 {self.video_device})")
                    return
                self.get_logger().info(f"Opened {self.video_device} with resolution {w}x{h} at {self.fps} FPS")

            self.writer.write(cv_image)
        except Exception as e:
            self.get_logger().error(f"Failed to write frame to loopback device: {e}")

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f"Released {self.video_device}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KinectToV4L2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
