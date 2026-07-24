#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import math
from sensor_msgs.msg import Imu, MagneticField

class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')

        self.declare_parameter('port', '/dev/razor')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('use_ned', True)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.use_ned = self.get_parameter('use_ned').value

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info(f"Connected to IMU on {self.port} at {self.baud} baud.")
        except Exception as e:
            self.get_logger().error(f"Could not open port {self.port}: {e}")
            raise e

        self.create_timer(0.01, self.read_serial)

    def read_serial(self):
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
