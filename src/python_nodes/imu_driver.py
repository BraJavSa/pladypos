#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import math
import os
import glob
import time
import threading
from sensor_msgs.msg import Imu, MagneticField

class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value

        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)

        self.ser = None
        self._stop = threading.Event()
        self._thread = None

        self.start()

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2)

    def connect_serial(self):
        try:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Conectado a la IMU en puerto {self.port} @ {self.baud} baud.")
            return True
        except Exception as e:
            self.get_logger().warn(f"No se pudo abrir puerto {self.port}: {e}")
            self.ser = None
            return False

    def find_imu_port(self):
        by_id_path = '/dev/serial/by-id'
        by_id_map = {}
        if os.path.exists(by_id_path):
            for f in os.listdir(by_id_path):
                path = os.path.join(by_id_path, f)
                real = os.path.realpath(path)
                by_id_map[real] = f.lower()

        arduino_port = None
        for port, name in by_id_map.items():
            if 'micro' in name or 'arduino' in name:
                arduino_port = port
                break

        for port, name in by_id_map.items():
            if port != arduino_port:
                if 'razor' in name or 'ftdi' in name or 'usb-uart' in name or 'usb' in name:
                    return port

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

    def _run(self):
        """Hilo dedicado a la lectura continua del puerto serie a la frecuencia nativa del sensor."""
        while not self._stop.is_set():
            if self.ser is None or not self.ser.is_open:
                target_port = self.port
                if target_port == 'auto':
                    target_port = self.find_imu_port()

                if target_port:
                    self.port = target_port
                    self.connect_serial()
                else:
                    time.sleep(1.0)
                    continue

            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                if ',' in line:
                    parts = line.split(',')
                    if len(parts) >= 10:
                        try:
                            raw_acc = [float(x) for x in parts[1:4]]
                            raw_gyro = [float(x) for x in parts[4:7]]
                            raw_mag = [float(x) for x in parts[7:10]]
                        except ValueError:
                            continue

                        # Rotación 180° sobre el eje Y: (x -> -x, y -> y, z -> -z)
                        acc_x = -raw_acc[0]
                        acc_y =  raw_acc[1]
                        acc_z = -raw_acc[2]

                        gyro_x = -raw_gyro[0]
                        gyro_y =  raw_gyro[1]
                        gyro_z = -raw_gyro[2]

                        mag_x = -raw_mag[0]
                        mag_y =  raw_mag[1]
                        mag_z = -raw_mag[2]

                        stamp = self.get_clock().now().to_msg()

                        # Mensaje Imu
                        imu_msg = Imu()
                        imu_msg.header.stamp = stamp
                        imu_msg.header.frame_id = self.frame_id

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

                        # Conversión a m/s² y rad/s
                        imu_msg.linear_acceleration.x = acc_x * 9.8065
                        imu_msg.linear_acceleration.y = acc_y * 9.8065
                        imu_msg.linear_acceleration.z = acc_z * 9.8065

                        imu_msg.angular_velocity.x = gyro_x * math.pi / 180.0
                        imu_msg.angular_velocity.y = gyro_y * math.pi / 180.0
                        imu_msg.angular_velocity.z = gyro_z * math.pi / 180.0

                        self.imu_pub.publish(imu_msg)

                        # Mensaje Campo Magnético (Teslas)
                        mag_msg = MagneticField()
                        mag_msg.header.stamp = stamp
                        mag_msg.header.frame_id = self.frame_id
                        mag_msg.magnetic_field.x = mag_x * 1e-7
                        mag_msg.magnetic_field.y = mag_y * 1e-7
                        mag_msg.magnetic_field.z = mag_z * 1e-7

                        self.mag_pub.publish(mag_msg)

            except Exception as e:
                self.get_logger().warn(f"Error en lectura serie IMU: {e}")
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = None
                time.sleep(1.0)

    def destroy_node(self):
        self.stop()
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