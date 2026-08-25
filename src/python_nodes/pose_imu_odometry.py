#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
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
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('invert_yaw', True)
        self.declare_parameter('invert_z', True)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.invert_yaw = self.get_parameter('invert_yaw').value
        self.invert_z = self.get_parameter('invert_z').value

        # Publicador de Odometría
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Suscriptores a todas las variantes de tópicos de IMU y Pose
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10)
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

        self.get_logger().info(
            f"Nodo pose_imu_odometry iniciado.\n"
            f"  Pose: {self.pose_topic}\n"
            f"  IMU: {self.imu_topic}\n"
            f"  Odometría: {self.odom_topic} @ {self.rate} Hz\n"
            f"  TF: {self.frame_id} -> {self.child_frame_id}"
        )

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.pose_count += 1
        now = self.get_clock().now().nanoseconds / 1e9
        if self.latest_pose is not None and self.last_pose_time is not None:
            dt = now - self.last_pose_time
            if dt > 0.001:
                dx = msg.pose.pose.position.x - self.latest_pose.pose.pose.position.x
                dy = msg.pose.pose.position.y - self.latest_pose.pose.pose.position.y
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
                wz = -msg.angular_velocity.z if self.invert_yaw else msg.angular_velocity.z
                self.yaw += wz * dt

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
            odom_msg.pose.pose.position.x = self.latest_pose.pose.pose.position.x
            odom_msg.pose.pose.position.y = self.latest_pose.pose.pose.position.y
            odom_msg.pose.pose.position.z = 0.0
        else:
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0

        # Orientación y Yaw actual en sistema FLU (Invertido 180° para Z arriba)
        self.yaw_flu = -self.yaw
        half_yaw = self.yaw_flu * 0.5

        if self.latest_imu is not None and not (self.latest_imu.orientation.w == 0.0 and self.latest_imu.orientation.x == 0.0):
            ori = self.latest_imu.orientation
            # Pasar orientación de la IMU tal cual (sin forzar Z hacia arriba)
            odom_msg.pose.pose.orientation.w = ori.w
            odom_msg.pose.pose.orientation.x = ori.x
            odom_msg.pose.pose.orientation.y = ori.y
            odom_msg.pose.pose.orientation.z = ori.z

            o = odom_msg.pose.pose.orientation
            siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
            cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)
            odom_msg.twist.twist.angular.z = -self.latest_imu.angular_velocity.z
        else:
            odom_msg.pose.pose.orientation.w = math.cos(half_yaw)
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = math.sin(half_yaw)
            current_yaw = self.yaw_flu
            if self.latest_imu is not None:
                odom_msg.twist.twist.angular.z = -self.latest_imu.angular_velocity.z

        # Velocidad lineal en el marco FLU del vehículo (X = Frente, Y = Izquierda)
        cos_y = math.cos(current_yaw)
        sin_y = math.sin(current_yaw)
        odom_msg.twist.twist.linear.x = self.vx * cos_y + self.vy * sin_y
        odom_msg.twist.twist.linear.y = -self.vx * sin_y + self.vy * cos_y
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
