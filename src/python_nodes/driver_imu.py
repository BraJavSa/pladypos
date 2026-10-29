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

import glob
import math
import os
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

        self.declare_parameter("port", "auto")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("mag_offset_x", DEFAULT_MAG_OFFSET_X)
        self.declare_parameter("mag_offset_y", DEFAULT_MAG_OFFSET_Y)
        self.declare_parameter("mag_offset_z", DEFAULT_MAG_OFFSET_Z)

        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baud").value
        self.ox = self.get_parameter("mag_offset_x").value
        self.oy = self.get_parameter("mag_offset_y").value
        self.oz = self.get_parameter("mag_offset_z").value
        self.get_logger().info(
            f"Magnetometer offsets: x={self.ox}, y={self.oy}, z={self.oz}"
        )

        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)

        self.ser = None
        self.last_reconnect_time = 0.0

        if self.port == "auto":
            self.get_logger().info("Searching for IMU port dynamically...")
            detected = self.find_imu_port()
            if detected:
                self.port = detected
                self.connect_serial()
            else:
                self.get_logger().warn("IMU not found on startup, will keep scanning...")
        else:
            self.connect_serial()

        # Fast polling timer: read and process one line per tick
        self.timer = self.create_timer(0.005, self.tick)

    def connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.get_logger().info(f"Opening serial port {self.port} @ {self.baud}...")
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Connected to IMU on {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().warn(f"Could not open serial port {self.port}: {e}")
            self.ser = None

    def find_imu_port(self):
        by_id_path = "/dev/serial/by-id"
        by_id_map = {}
        if os.path.exists(by_id_path):
            for f in os.listdir(by_id_path):
                path = os.path.join(by_id_path, f)
                real = os.path.realpath(path)
                by_id_map[real] = f.lower()

        # 1. Identify Arduino port to avoid conflicting with serial_bridge
        arduino_port = None
        for port, name in by_id_map.items():
            if "micro" in name or "arduino" in name:
                arduino_port = port
                break

        # 2. Check for Razor / SparkFun / SAMD devices in by-id
        for port, name in by_id_map.items():
            if port != arduino_port:
                if any(k in name for k in ["razor", "sparkfun", "samd", "ftdi", "usb-uart", "usb"]):
                    return port

        # 3. Fallback: scan serial ports
        candidates = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")
        candidates = sorted(list(set([os.path.realpath(c) for c in candidates])))

        for port in candidates:
            if port == arduino_port:
                continue
            try:
                test_ser = serial.Serial(port, self.baud, timeout=0.3)
                time.sleep(0.1)
                line = test_ser.readline().decode("utf-8", errors="ignore").strip()
                test_ser.close()
                if line and len(line.split(",")) == 10:
                    return port
            except Exception:
                continue
        return None

    def tick(self):
        if not self.ser or not self.ser.is_open:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_reconnect_time > 2.0:
                self.last_reconnect_time = now
                if self.get_parameter("port").value == "auto":
                    detected = self.find_imu_port()
                    if detected:
                        self.port = detected
                        self.connect_serial()
                else:
                    self.connect_serial()
            return

        try:
            if self.ser.in_waiting == 0:
                return
            raw = self.ser.readline().decode(errors="ignore")
        except Exception as e:
            self.get_logger().warn(f"Error reading IMU serial port: {e}")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.last_reconnect_time = self.get_clock().now().nanoseconds / 1e9
            return
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

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


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