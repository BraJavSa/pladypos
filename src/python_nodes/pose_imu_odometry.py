#!/usr/bin/env python3
"""
pose_imu_odometry.py
Nodo ROS 2 que fusiona:
- Posición 3D (x, y, z) obtenida de la odometría de AprilTag (/usv5/odom)
- Orientación (cuaternión / yaw) obtenida del tópico de la IMU (/imu/data_raw)

Publica Odometría (nav_msgs/Odometry) y la transformada TF para 'imubased_usv5' a 20 Hz.
"""

import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class PoseImuOdometryNode(Node):
    def __init__(self):
        super().__init__('pose_imu_odometry')

        # Declaración de parámetros ROS 2
        self.declare_parameter('apriltag_odom_topic', '/usv5/odom')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/usv5/imubased_odom')
        self.declare_parameter('frame_id', 'camera')
        self.declare_parameter('child_frame_id', 'imubased_usv5')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('publish_tf', True)

        self.apriltag_odom_topic = self.get_parameter('apriltag_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.rate = float(self.get_parameter('publish_rate').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        # Publicador de odometría fusionada
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Suscripciones: Posición (AprilTag) e IMU (Orientación)
        self.create_subscription(Odometry, self.apriltag_odom_topic, self.apriltag_odom_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 10)

        # Estado guardado de posición (AprilTag) e imu (orientación)
        self.latest_pos = None     # (x, y, z, timestamp)
        self.prev_pos = None       # (x, y, z, timestamp)
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        self.latest_ori = (1.0, 0.0, 0.0, 0.0)  # (qw, qx, qy, qz)
        self.latest_imu_time = None
        self.last_imu_step_time = None
        self.yaw = 0.0
        self.angular_vel_z = 0.0

        # Timer a frecuencia fija (20 Hz)
        timer_period = 1.0 / self.rate
        self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"Nodo pose_imu_odometry iniciado:\n"
            f"  Posición XYZ de: {self.apriltag_odom_topic}\n"
            f"  Orientación de: {self.imu_topic}\n"
            f"  Odometría publicada en: {self.odom_topic}\n"
            f"  TF: {self.frame_id} -> {self.child_frame_id} @ {self.rate} Hz"
        )

    def apriltag_odom_cb(self, msg: Odometry):
        """Toma ÚNICAMENTE la posición 3D (x, y, z) de la odometría del AprilTag."""
        now_sec = time.time()
        pos = msg.pose.pose.position
        x, y, z = pos.x, pos.y, pos.z

        if self.prev_pos is not None:
            px, py, pz, pt = self.prev_pos
            dt = now_sec - pt
            if dt > 0.001:
                self.vx = (x - px) / dt
                self.vy = (y - py) / dt
                self.vz = (z - pz) / dt

        self.prev_pos = (x, y, z, now_sec)
        self.latest_pos = (x, y, z, now_sec)

    def imu_cb(self, msg: Imu):
        """Toma ÚNICAMENTE la orientación de la IMU."""
        now_sec = time.time()
        ori = msg.orientation
        self.angular_vel_z = msg.angular_velocity.z

        # Si la IMU proporciona cuaternión de orientación válido
        if not (ori.w == 0.0 and ori.x == 0.0 and ori.y == 0.0 and ori.z == 0.0):
            self.latest_ori = (ori.w, ori.x, ori.y, ori.z)
        else:
            # Si son lecturas de IMU sin cuaternión directo, integramos la velocidad angular (yaw)
            if self.last_imu_step_time is not None:
                dt = now_sec - self.last_imu_step_time
                if 0.0001 < dt < 1.0:
                    self.yaw += msg.angular_velocity.z * dt
                    # Normalizar yaw a [-pi, pi]
                    self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi

            half_yaw = self.yaw * 0.5
            self.latest_ori = (math.cos(half_yaw), 0.0, 0.0, math.sin(half_yaw))

        self.last_imu_step_time = now_sec
        self.latest_imu_time = now_sec

    def timer_callback(self):
        """Calcula y publica la odometría fusionada y la TF para 'imubased_usv5' a 20 Hz."""
        stamp = self.get_clock().now().to_msg()
        now_sec = time.time()

        # Determinar posición 3D de AprilTag (o cero si no hay datos recientmente)
        if self.latest_pos is not None and (now_sec - self.latest_pos[3]) <= 1.0:
            x, y, z, _ = self.latest_pos
            vx, vy, vz = self.vx, self.vy, self.vz
        else:
            x, y, z = 0.0, 0.0, 0.0
            vx, vy, vz = 0.0, 0.0, 0.0

        # Determinar orientación tomada exclusivamente de la IMU
        qw, qx, qy, qz = self.latest_ori

        # 1. Publicar nav_msgs/Odometry fusionada
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z

        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz

        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = vz
        odom_msg.twist.twist.angular.z = self.angular_vel_z

        odom_msg.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        self.odom_pub.publish(odom_msg)

        # 2. Publicar Transformada TF (camera -> imubased_usv5)
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.frame_id
            tf_msg.child_frame_id = self.child_frame_id

            tf_msg.transform.translation.x = x
            tf_msg.transform.translation.y = y
            tf_msg.transform.translation.z = z

            tf_msg.transform.rotation.w = qw
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz

            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PoseImuOdometryNode()
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
