#!/usr/bin/env python3
import sys
import os
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        
        # Parameters
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        device = self.get_parameter('video_device').value
        fps = self.get_parameter('frame_rate').value
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
        
        self.get_logger().info(f"Opening video device: {device}")
        
        # Try to parse device as integer if it represents index (e.g. "0")
        try:
            dev_id = int(device)
        except ValueError:
            dev_id = device
            
        self.cap = cv2.VideoCapture(dev_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video device: {device}")
            self.get_logger().info("Trying fallback to device index 0...")
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                self.get_logger().error("Fallback failed. No camera available.")
                
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.get_logger().info("Camera initialized successfully.")
            
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def timer_callback(self):
        if not self.cap.isOpened():
            return
            
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to grab frame.")
            return
            
        stamp = self.get_clock().now().to_msg()
        
        # Publish raw image
        try:
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            raw_msg.header.stamp = stamp
            raw_msg.header.frame_id = "camera_link"
            self.image_pub.publish(raw_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish raw image: {e}")
            
        # Publish compressed image (JPEG)
        try:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = stamp
            compressed_msg.header.frame_id = "camera_link"
            compressed_msg.format = "jpeg"
            
            # Encode frame to JPEG with 80% quality
            ret_encode, jpeg_data = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ret_encode:
                compressed_msg.data = jpeg_data.tobytes()
                self.compressed_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish compressed image: {e}")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
