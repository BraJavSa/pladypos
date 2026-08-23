#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import math
import os
import glob
import time
from sensor_msgs.msg import Imu, MagneticField

class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('use_ned', False)
        self.declare_parameter('use_flu', True)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.use_ned = self.get_parameter('use_ned').value
        self.use_flu = self.get_parameter('use_flu').value and not self.use_ned

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        self.ser = None
        self.last_reconnect_time = 0.0

        if self.port == 'auto':
            self.get_logger().info("Searching for IMU port dynamically...")
            detected_port = self.find_imu_port()
            if detected_port:
                self.port = detected_port
                self.connect_serial()
            else:
                self.get_logger().warn("Could not detect IMU on startup, will keep scanning...")
        else:
            self.connect_serial()

        self.create_timer(0.005, self.read_serial)

    def connect_serial(self):
        try:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            frame_str = "NED" if self.use_ned else "FLU (Forward, Left, Up)"
            self.get_logger().info(f"Connected to IMU on {self.port} at {self.baud} baud [{frame_str}].")
        except Exception as e:
            self.get_logger().warn(f"Could not open IMU port {self.port}: {e}")
            self.ser = None

    def recover_serial(self):
        if self.port == 'auto':
            self.get_logger().info("Scanning for IMU port to reconnect...")
            detected_port = self.find_imu_port()
            if detected_port:
                self.port = detected_port
                self.connect_serial()
        else:
            self.get_logger().info(f"Attempting to reconnect to IMU port {self.port}...")
            self.connect_serial()

    def find_imu_port(self):
        by_id_path = '/dev/serial/by-id'
        by_id_ports = []
        by_id_map = {}
        if os.path.exists(by_id_path):
            for f in os.listdir(by_id_path):
                path = os.path.join(by_id_path, f)
                real = os.path.realpath(path)
                by_id_ports.append(real)
                by_id_map[real] = f.lower()

        # 1. Try by-id names first (very reliable, avoids double-open)
        arduino_port = None
        for port, name in by_id_map.items():
            if 'micro' in name or 'arduino' in name:
                arduino_port = port
                break
                
        for port, name in by_id_map.items():
            if port != arduino_port:
                if 'razor' in name or 'ftdi' in name or 'usb-uart' in name or 'usb' in name:
                    return port

        # 2. Fallback to candidate scan
        candidates = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        candidates = sorted(list(set([os.path.realpath(c) for c in candidates])))
        
        for port in candidates:
            if port == arduino_port:
                continue
            try:
                ser = serial.Serial(port, self.baud, timeout=0.3)
                time.sleep(0.1)
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                ser.close()
                if ',' in line and len(line.split(',')) >= 10:
                    return port
            except Exception:
                continue
        return None

    def read_serial(self):
        if not self.ser or not self.ser.is_open:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_reconnect_time > 1.5:
                self.last_reconnect_time = now
                self.recover_serial()
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if ',' in line:
                    parts = line.split(',')
                    if len(parts) >= 10:
                        try:
                            acc = [float(x) for x in parts[1:4]]
                            gyro = [float(x) for x in parts[4:7]]
                            mag = [float(x) for x in parts[7:10]]
                        except ValueError as ve:
                            self.get_logger().debug(f"Skipping corrupt IMU line: {line} ({ve})")
                            return

                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = 'imu_link'
                        # Non-zero covariances for robot_localization EKF fusion
                        imu_msg.orientation_covariance = [
                            0.001, 0.0, 0.0,
                            0.0, 0.001, 0.0,
                            0.0, 0.0, 0.001
                        ]
                        imu_msg.angular_velocity_covariance = [
                            0.0001, 0.0, 0.0,
                            0.0, 0.0001, 0.0,
                            0.0, 0.0, 0.0001
                        ]
                        imu_msg.linear_acceleration_covariance = [
                            0.01, 0.0, 0.0,
                            0.0, 0.01, 0.0,
                            0.0, 0.0, 0.01
                        ]

                        if self.use_ned:
                            imu_msg.linear_acceleration.x = -acc[0] * 9.8065
                            imu_msg.linear_acceleration.y = -acc[1] * 9.8065
                            imu_msg.linear_acceleration.z = acc[2] * 9.8065
                            imu_msg.angular_velocity.x = -gyro[0] * math.pi / 180.0
                            imu_msg.angular_velocity.y = -gyro[1] * math.pi / 180.0
                            imu_msg.angular_velocity.z = gyro[2] * math.pi / 180.0
                        else:
                            # FLU Frame: Rotación 180° en X seguida de +270° en Z
                            imu_msg.linear_acceleration.x = -acc[1] * 9.8065
                            imu_msg.linear_acceleration.y = -acc[0] * 9.8065
                            imu_msg.linear_acceleration.z = -acc[2] * 9.8065
                            imu_msg.angular_velocity.x = -gyro[1] * math.pi / 180.0
                            imu_msg.angular_velocity.y = -gyro[0] * math.pi / 180.0
                            imu_msg.angular_velocity.z = -gyro[2] * math.pi / 180.0

                        self.imu_pub.publish(imu_msg)

                        mag_msg = MagneticField()
                        mag_msg.header.stamp = imu_msg.header.stamp
                        mag_msg.header.frame_id = 'imu_link'
                        mag_msg.magnetic_field.x = -mag[1] * 1e-7 if not self.use_ned else -mag[1] * 1e-7
                        mag_msg.magnetic_field.y = -mag[0] * 1e-7 if not self.use_ned else -mag[0] * 1e-7
                        mag_msg.magnetic_field.z = -mag[2] * 1e-7 if not self.use_ned else -mag[2] * 1e-7

                        self.mag_pub.publish(mag_msg)

        except Exception as e:
            self.get_logger().warn(f"Error reading serial port: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.last_reconnect_time = self.get_clock().now().nanoseconds / 1e9

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IMUDriver()
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
