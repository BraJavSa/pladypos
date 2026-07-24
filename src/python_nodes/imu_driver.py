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
        self.declare_parameter('use_ned', True)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.use_ned = self.get_parameter('use_ned').value

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

        self.create_timer(0.01, self.read_serial)

    def connect_serial(self):
        try:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Connected to IMU on {self.port} at {self.baud} baud.")
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
        if os.path.exists(by_id_path):
            for f in os.listdir(by_id_path):
                if 'razor' in f.lower() or 'ftdi' in f.lower() or 'usb-uart' in f.lower():
                    path = os.path.join(by_id_path, f)
                    by_id_ports.append(os.path.realpath(path))
                    
        candidates = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        candidates = sorted(list(set([os.path.realpath(c) for c in candidates] + by_id_ports)))

        # Check by-id names first for 'razor'
        if os.path.exists(by_id_path):
            for f in os.listdir(by_id_path):
                if 'razor' in f.lower():
                    return os.path.realpath(os.path.join(by_id_path, f))

        for port in candidates:
            # Skip Arduino Micro/Arduino ports based on name
            if 'micro' in port.lower() or 'arduino' in port.lower():
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
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if ',' in line:
                    parts = line.split(',')
                    if len(parts) >= 10:
                        acc = [float(x) for x in parts[1:4]]
                        gyro = [float(x) for x in parts[4:7]]
                        mag = [float(x) for x in parts[7:10]]

                        imu_msg = Imu()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = 'imu_link'

                        if self.use_ned:
                            imu_msg.linear_acceleration.x = -acc[0] * 9.8065
                            imu_msg.linear_acceleration.y = -acc[1] * 9.8065
                            imu_msg.linear_acceleration.z = acc[2] * 9.8065
                            imu_msg.angular_velocity.x = -gyro[0] * math.pi / 180.0
                            imu_msg.angular_velocity.y = -gyro[1] * math.pi / 180.0
                            imu_msg.angular_velocity.z = gyro[2] * math.pi / 180.0
                        else:
                            imu_msg.linear_acceleration.x = -acc[0] * 9.8065
                            imu_msg.linear_acceleration.y = acc[1] * 9.8065
                            imu_msg.linear_acceleration.z = -acc[2] * 9.8065
                            imu_msg.angular_velocity.x = -gyro[0] * math.pi / 180.0
                            imu_msg.angular_velocity.y = gyro[1] * math.pi / 180.0
                            imu_msg.angular_velocity.z = -gyro[2] * math.pi / 180.0

                        self.imu_pub.publish(imu_msg)

                        mag_msg = MagneticField()
                        mag_msg.header.stamp = imu_msg.header.stamp
                        mag_msg.header.frame_id = 'imu_link'
                        mag_msg.magnetic_field.x = -mag[1] * 1e-7
                        mag_msg.magnetic_field.y = -mag[0] * 1e-7 if self.use_ned else mag[0] * 1e-7
                        mag_msg.magnetic_field.z = -mag[2] * 1e-7 if self.use_ned else mag[2] * 1e-7

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
