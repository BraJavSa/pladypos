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

    # Matriz de rotación 180° alrededor del eje Y
    # Rot_y(180) = [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]
    ROT_Y_180 = (
        (-1.0, 0.0,  0.0),
        ( 0.0, 1.0,  0.0),
        ( 0.0, 0.0, -1.0),
    )

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

    @staticmethod
    def rotate_vector(v, R):
        """Multiplica el vector v (largo 3) por la matriz de rotación R (3x3)."""
        return [
            R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
        ]

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
            self.ser = serial.Serial(self.port, self.baud, timeout=2.0)
            # Enviar comando de inicio de streaming continuo al firmware Razor si aplica
            try:
                self.ser.write(b"#o1\n")
            except Exception:
                pass
            self.get_logger().info(f"Conectado a la IMU en {self.port} @ {self.baud} baud (hilo dedicado a máxima frecuencia).")
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
        """Hilo dedicado a la lectura continua del puerto serie a la máxima tasa del sensor (50Hz+)."""
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
                            acc = [float(x) for x in parts[1:4]]
                            gyro = [float(x) for x in parts[4:7]]
                            mag = [float(x) for x in parts[7:10]]
                        except ValueError:
                            continue

                        # --- Rotación 180° en Y aplicada vía matriz a los 3 vectores ---
                        acc = self.rotate_vector(acc, self.ROT_Y_180)
                        gyro = self.rotate_vector(gyro, self.ROT_Y_180)
                        mag = self.rotate_vector(mag, self.ROT_Y_180)
                        # ----------------------------------------------------------------

                        stamp = self.get_clock().now().to_msg()

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

                        imu_msg.linear_acceleration.x = acc[0] * 9.8065
                        imu_msg.linear_acceleration.y = acc[1] * 9.8065
                        imu_msg.linear_acceleration.z = acc[2] * 9.8065

                        imu_msg.angular_velocity.x = gyro[0] * math.pi / 180.0
                        imu_msg.angular_velocity.y = gyro[1] * math.pi / 180.0
                        imu_msg.angular_velocity.z = gyro[2] * math.pi / 180.0

                        self.imu_pub.publish(imu_msg)

                        mag_msg = MagneticField()
                        mag_msg.header.stamp = stamp
                        mag_msg.header.frame_id = self.frame_id
                        mag_msg.magnetic_field.x = mag[0] * 1e-7
                        mag_msg.magnetic_field.y = mag[1] * 1e-7
                        mag_msg.magnetic_field.z = mag[2] * 1e-7

                        self.mag_pub.publish(mag_msg)

            except Exception as e:
                self.get_logger().warn(f"Error en lectura serie: {e}")
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