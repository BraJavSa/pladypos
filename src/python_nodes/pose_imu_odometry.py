#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf2_ros

class PoseImuOdometryNode(Node):
    def __init__(self):
        super().__init__('pose_imu_odometry')

        # Parámetros
        self.declare_parameter('pose_topic', '/usv5/pose')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/usv5/odom')
        self.declare_parameter('publish_rate', 10.0)  # 10 Hz
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'usv5')
        self.declare_parameter('publish_tf', True)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # Publicador de Odometría
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Suscriptores a todas las variantes de tópicos de IMU y Pose
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_raw_callback, 10)
        self.create_subscription(Imu, '/usv5/imu/data', self.imu_callback, 10)
        self.create_subscription(Imu, '/usv5/imu/data_raw', self.imu_raw_callback, 10)

        # Estado guardado
        self.latest_pose = None
        self.latest_imu = None
        self.last_pose_time = None
        self.last_imu_time = None
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.pose_count = 0
        self.imu_count = 0

        # Timer a 10 Hz
        timer_period = 1.0 / self.rate
        self.create_timer(timer_period, self.timer_callback)
        self.create_timer(3.0, self.diag_callback)

        self.get_logger().info(
            f"Nodo pose_imu_odometry iniciado.\n"
            f"  Pose: {self.pose_topic}\n"
            f"  IMU: {self.imu_topic} (/usv5/imu/data_raw)\n"
            f"  Odometría: {self.odom_topic} @ {self.rate} Hz\n"
            f"  TF: {self.frame_id} -> {self.child_frame_id}"
        )

    def diag_callback(self):
        if self.pose_count == 0:
            self.get_logger().warn(f"Esperando datos en {self.pose_topic} (AprilTag no detectado aún)...")
        else:
            self.get_logger().info(f"Pose OK! Recibidos {self.pose_count} mensajes de pose.")

        if self.imu_count == 0:
            self.get_logger().warn("Esperando datos de la IMU en /imu/data o /usv5/imu/data_raw...")
        else:
            self.get_logger().info(f"IMU OK! Recibidos {self.imu_count} mensajes de IMU. Yaw actual: {math.degrees(self.yaw):.2f}°")

    def pose_callback(self, msg: PoseStamped):
        self.pose_count += 1
        now = self.get_clock().now().nanoseconds / 1e9
        if self.latest_pose is not None and self.last_pose_time is not None:
            dt = now - self.last_pose_time
            if dt > 0.001:
                dx = msg.pose.position.x - self.latest_pose.pose.position.x
                dy = msg.pose.position.y - self.latest_pose.pose.position.y
                self.vx = dx / dt
                self.vy = dy / dt

        self.latest_pose = msg
        self.last_pose_time = now

    def process_imu(self, msg: Imu):
        self.imu_count += 1
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_imu_time is not None:
            dt = now - self.last_imu_time
            if 0.0001 < dt < 1.0:
                self.yaw += msg.angular_velocity.z * dt

        self.last_imu_time = now
        self.latest_imu = msg

    def imu_callback(self, msg: Imu):
        self.process_imu(msg)

    def imu_raw_callback(self, msg: Imu):
        if self.latest_imu is None or (self.latest_imu.orientation.w == 0.0 and self.latest_imu.orientation.x == 0.0):
            self.process_imu(msg)

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        # Covarianzas por defecto
        odom_msg.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        odom_msg.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]

        # Posición desde /usv5/pose (X, Y, Z=0)
        if self.latest_pose is not None:
            odom_msg.pose.pose.position.x = self.latest_pose.pose.position.x
            odom_msg.pose.pose.position.y = self.latest_pose.pose.position.y
            odom_msg.pose.pose.position.z = 0.0
        else:
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0

        # Orientación y Velocidad Angular desde la IMU (o integración de Yaw de respaldo)
        if self.latest_imu is not None:
            ori = self.latest_imu.orientation
            if ori.w == 0.0 and ori.x == 0.0 and ori.y == 0.0 and ori.z == 0.0:
                # Calcular orientación cuaternión desde el Yaw integrado
                half_yaw = self.yaw * 0.5
                odom_msg.pose.pose.orientation.w = math.cos(half_yaw)
                odom_msg.pose.pose.orientation.x = 0.0
                odom_msg.pose.pose.orientation.y = 0.0
                odom_msg.pose.pose.orientation.z = math.sin(half_yaw)
            else:
                odom_msg.pose.pose.orientation = ori
            odom_msg.twist.twist.angular = self.latest_imu.angular_velocity
        else:
            half_yaw = self.yaw * 0.5
            odom_msg.pose.pose.orientation.w = math.cos(half_yaw)
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = math.sin(half_yaw)

        # Velocidad lineal estimada
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.linear.z = 0.0

        self.odom_pub.publish(odom_msg)

        # Publicar TF (odom -> usv5)
        if self.publish_tf and rclpy.ok():
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.frame_id
            tf_msg.child_frame_id = self.child_frame_id
            tf_msg.transform.translation.x = float(odom_msg.pose.pose.position.x)
            tf_msg.transform.translation.y = float(odom_msg.pose.pose.position.y)
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.x = float(odom_msg.pose.pose.orientation.x)
            tf_msg.transform.rotation.y = float(odom_msg.pose.pose.orientation.y)
            tf_msg.transform.rotation.z = float(odom_msg.pose.pose.orientation.z)
            tf_msg.transform.rotation.w = float(odom_msg.pose.pose.orientation.w)
            try:
                self.tf_broadcaster.sendTransform(tf_msg)
            except Exception as e:
                self.get_logger().error(f"Error publicando TF: {e}")

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
