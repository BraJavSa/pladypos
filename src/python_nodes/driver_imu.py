#!/usr/bin/env python3
"""
Driver serial para la SparkFun 9DoF Razor IMU M0 (ROS2 Jazzy).

No calcula orientacion: solo lee el serial y publica los datos crudos
(accel+gyro y magnetometro ya calibrado con hard-iron), para que
imu_filter_madgwick haga la fusion.

Publica:
  - imu/data_raw   (sensor_msgs/Imu)           accel [m/s^2] + gyro [rad/s]
                    orientation sin usar (covarianza[0] = -1)
  - imu/mag        (sensor_msgs/MagneticField)  magnetometro calibrado

CALIBRACION YA APLICADA POR DEFECTO:
  - Offsets de hard-iron del magnetometro: mag_offset_x=-1.95,
    mag_offset_y=6.90, mag_offset_z=-8.325 (obtenidos con calibrate_mag.py).
  - Signo de az invertido (az_fix = -az): en este montaje fisico boca-arriba
    y en reposo el eje Z crudo da ~-9.8 m/s^2 en vez de +9.8.
  Todos son parametros ROS2 y se pueden sobreescribir desde el launch.

Parametros ROS2:
  port          (string, default '/dev/ttyACM0')
  baud          (int,    default 115200)
  mag_offset_x  (double, default -1.95)
  mag_offset_y  (double, default 6.90)
  mag_offset_z  (double, default -8.325)

Requiere:
    pip3 install pyserial --break-system-packages
"""

import math
import time

import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

# --- Calibracion de magnetometro (hard-iron), obtenida el 2026-08-29 ---
DEFAULT_MAG_OFFSET_X = -1.950000
DEFAULT_MAG_OFFSET_Y = 6.900000
DEFAULT_MAG_OFFSET_Z = -8.325000

# El magnetometro de esta IMU entrega datos en unidades relativas (gauss
# aproximado / counts, segun firmware). sensor_msgs/MagneticField espera
# Tesla. Para la fusion de Madgwick basta con que sea proporcional, ya
# que el filtro normaliza el vector internamente.
MAG_TO_TESLA = 1e-4  # asume entrada aproximada en Gauss -> Tesla


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
            f"Offsets de magnetometro: x={self.ox}, y={self.oy}, z={self.oz}"
        )

        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag", 10)

        self.get_logger().info(f"Abriendo puerto serie {port} @ {baud}...")
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        self.ser.reset_input_buffer()

        # Timer rapido: en cada tick intenta leer y procesar una linea
        self.timer = self.create_timer(0.005, self.tick)

    def tick(self):
        if self.ser.in_waiting == 0:
            return
        raw = self.ser.readline().decode(errors="ignore")
        parsed = parse_line(raw)
        if parsed is None:
            return

        _t_ms, ax, ay, az, gx, gy, gz, mx, my, mz = parsed

        # Correccion de signo de Z: en este montaje fisico, boca-arriba
        # y en reposo el eje Z crudo da ~-1g en vez de +1g.
        az = -az

        # Correccion de quiralidad del giroscopio: yaw y pitch salian
        # invertidos (giran en sentido contrario al movimiento real)
        # mientras que roll estaba correcto. Esto es tipico cuando el
        # giroscopio entrega los ejes en convencion left-hand en vez
        # de la right-hand que espera Madgwick (world_frame=enu).
        # Se invierten gy y gz (no gx, que corresponde a roll y ya
        # estaba bien) para corregir el sentido de rotacion.
        gy = -gy
        gz = -gz

        mx -= self.ox
        my -= self.oy
        mz -= self.oz

        now = self.get_clock().now().to_msg()

        # --- Publicar sensor_msgs/Imu crudo (sin orientacion) ---
        msg = Imu()
        msg.header.stamp = now
        msg.header.frame_id = "imu_link"

        # orientation desconocida: covarianza[0] = -1 le dice a los
        # consumidores (como imu_filter_madgwick) que la ignoren.
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)

        # accel cruda viene en 'g'; sensor_msgs/Imu espera m/s^2
        msg.linear_acceleration.x = ax * 9.80665
        msg.linear_acceleration.y = ay * 9.80665
        msg.linear_acceleration.z = az * 9.80665

        self.imu_pub.publish(msg)

        # --- Publicar sensor_msgs/MagneticField ---
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