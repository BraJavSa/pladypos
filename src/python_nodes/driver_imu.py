#!/usr/bin/env python3
"""
Serial driver for SparkFun 9DoF Razor IMU M0 (ROS 2 Jazzy).

Does not compute orientation internally: reads raw serial data and publishes
raw accelerometer + gyroscope and hard-iron calibrated magnetometer data,
allowing imu_filter_madgwick to perform the sensor fusion.

Publishes:
  - imu/data_raw   (sensor_msgs/Imu)            accel [m/s^2] + gyro [rad/s]
                    orientation unused (covariance[0] = -1)
  - imu/mag        (sensor_msgs/MagneticField)  calibrated magnetometer

DEFAULT CALIBRATION APPLIED:
  - Magnetometer hard-iron offsets: mag_offset_x=-1.95,
    mag_offset_y=6.90, mag_offset_z=-8.325 (obtained with calibrate_mag.py).
  - Inverted az sign (az_fix = -az): in this physical face-up mounting
    at rest, raw Z reads ~ -9.8 m/s^2 instead of +9.8.
  All are ROS 2 parameters and can be overridden via launch or CLI.

ROS 2 Parameters:
  port          (string, default '/dev/ttyACM0')
  baud          (int,    default 115200)
  mag_offset_x  (double, default -1.95)
  mag_offset_y  (double, default 6.90)
  mag_offset_z  (double, default -8.325)

Requirements:
    pip3 install pyserial --break-system-packages
"""

import math
import time

import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

# --- Magnetometer hard-iron calibration (obtained 2026-08-29) ---
DEFAULT_MAG_OFFSET_X = -1.950000
DEFAULT_MAG_OFFSET_Y = 6.900000
DEFAULT_MAG_OFFSET_Z = -8.325000

# The magnetometer outputs relative units (approx Gauss / counts depending on firmware).
# sensor_msgs/MagneticField expects Tesla. For Madgwick fusion, proportional scaling
# is sufficient as the filter normalizes the vector internally.
MAG_TO_TESLA = 1e-4  # approx Gauss -> Tesla conversion


def parse_line(line):
    parts = line.strip().split(",")
    if len(parts) != 10:
        return None
    try:
        return tuple(float(p) for p in parts)
    except ValueError:
        return None


class ImuDriverNode(Node):
    def __init__(self):
        super().__init__("imu_driver_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("mag_offset_x", DEFAULT_MAG_OFFSET_X)
        self.declare_parameter("mag_offset_y", DEFAULT_MAG_OFFSET_Y)
        self.declare_parameter("mag_offset_z", DEFAULT_MAG_OFFSET_Z)

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        self.ox = self.get_parameter("mag_offset_x").value
        self.oy = self.get_parameter("mag_offset_y").value
        self.oz = self.get_parameter("mag_offset_z").value
        self.get_logger().info(
            f"Magnetometer offsets: x={self.ox}, y={self.oy}, z={self.oz}"
        )

        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)

        self.get_logger().info(f"Opening serial port {port} @ {baud}...")
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        self.ser.reset_input_buffer()

        # Fast polling timer: read and process one line per tick
        self.timer = self.create_timer(0.005, self.tick)

    def tick(self):
        if self.ser.in_waiting == 0:
            return
        raw = self.ser.readline().decode(errors="ignore")
        parsed = parse_line(raw)
        if parsed is None:
            return

        _t_ms, ax, ay, az, gx, gy, gz, mx, my, mz = parsed

        # Z-axis sign correction: in this face-up physical mounting at rest,
        # raw Z reads ~ -1g instead of +1g.
        az = -az

        # Gyroscope chirality correction: yaw and pitch were inverted
        # (rotating opposite to real motion) while roll was correct.
        # This occurs when gyro axes follow left-hand convention instead of
        # the right-hand ENU convention expected by Madgwick.
        # Invert gy and gz to correct rotation directions.
        gy = -gy
        gz = -gz

        mx -= self.ox
        my -= self.oy
        mz -= self.oz

        now = self.get_clock().now().to_msg()

        # --- Publish raw sensor_msgs/Imu (orientation uncalculated) ---
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = "imu_link"

        # Unknown orientation: covariance[0] = -1 instructs consumers
        # (e.g., imu_filter_madgwick) to ignore the orientation field.
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)

        # Raw accel is in 'g'; sensor_msgs/Imu expects m/s^2
        msg.linear_acceleration.x = ax * 9.80665
        msg.linear_acceleration.y = ay * 9.80665
        msg.linear_acceleration.z = az * 9.80665

        self.imu_pub.publish(msg)

        # --- Publish sensor_msgs/MagneticField ---
        mag_msg = MagneticField()
        mag_msg.header.stamp = now
        mag_msg.header.frame_id = "imu_link"
        mag_msg.magnetic_field.x = mx * MAG_TO_TESLA
        mag_msg.magnetic_field.y = my * MAG_TO_TESLA
        mag_msg.magnetic_field.z = mz * MAG_TO_TESLA
        self.mag_pub.publish(mag_msg)


def main():
    rclpy.init()
    node = ImuDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()