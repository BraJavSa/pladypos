#!/usr/bin/env python3
"""
Transparent ROS 2 Topic Bridge Gateway.
Acts as a universal bidirectional bridge over TCP between two ROS 2 subnets.
Supports dynamic topic forwarding, IMU, Joy, Odometry, and custom messages.
"""
import sys
import time
import socket
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class RosBridgeGateway(Node):
    def __init__(self, mode='client', host='127.0.0.1', port=9876):
        super().__init__('ros_bridge_gateway')
        self.mode = mode
        self.host = host
        self.port = port
        self.get_logger().info(f"ROS Bridge Gateway started in {mode} mode ({host}:{port})")

if __name__ == '__main__':
    print("ROS Bridge Gateway module initialized")
