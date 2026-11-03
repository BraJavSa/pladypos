#!/usr/bin/env python3
"""
direct_usv_fusion.py
Clean, robust, deterministic localization fusion for Pladypos USV.

Architecture:
1. Position (X, Y):
   - Direct from overhead AprilTag camera.
   - Z is strictly 0.0 (anchored to pool surface).
   - Smooth low-pass filter to eliminate pixel jitter.
   - NO dead-reckoning extrapolation during gaps (extrapolation causes uncontrolled flight).
     When camera is occluded, position holds steady at last known location.
2. Orientation (Roll, Pitch, Yaw):
   - Roll and Pitch are taken directly from the IMU (gravity vector is perfect).
   - Yaw is tracked continuously from IMU, with soft drift correction from camera.
3. Broadcasts TF:
   - odom -> usv5_odom (and odom -> usv5)
4. Publishes /usv5/odometry/filtered at 30 Hz.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_euler(w, x, y, z):
    """Returns (roll, pitch, yaw) in radians from unit quaternion."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quat(roll, pitch, yaw):
    """Converts (roll, pitch, yaw) to unit quaternion (w, x, y, z)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


class DirectUsvFusion(Node):
    def __init__(self):
        super().__init__('direct_usv_fusion')

        self.declare_parameter('visual_odom_topic', '/usv5/visual_odom')
        self.declare_parameter('imu_topic', '/usv5/imu/data')
        self.declare_parameter('odom_topic', '/usv5/odometry/filtered')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'usv5_odom')
        self.declare_parameter('water_depth', 5.1)
        self.declare_parameter('alpha_pos', 0.5)       # 0.5 gives responsive, smooth tracking
        self.declare_parameter('yaw_corr_rate', 0.05)  # Soft drift correction for IMU yaw

        self.visual_odom_topic = self.get_parameter('visual_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.water_depth = float(self.get_parameter('water_depth').value)
        self.alpha_pos = float(self.get_parameter('alpha_pos').value)
        self.yaw_corr_rate = float(self.get_parameter('yaw_corr_rate').value)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, self.visual_odom_topic, self.visual_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 20)

        # State Variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0  # ALWAYS 0.0
        self.has_visual = False

        # Orientation: Roll & Pitch directly from IMU, Yaw relative + camera correction
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_yaw = 0.0
        self.prev_imu_yaw = None
        self.fused_yaw = 0.0
        self.has_imu = False

        # Periodic 30 Hz timer
        self.create_timer(1.0 / 30.0, self.timer_tick)
        self.get_logger().info("Direct USV Fusion Node ready. Z locked at 0.0, IMU roll/pitch passthrough.")

    def imu_cb(self, msg: Imu):
        ori = msg.orientation
        if ori.w == 0.0 and ori.x == 0.0 and ori.y == 0.0 and ori.z == 0.0:
            return

        r, p, y = quat_to_euler(ori.w, ori.x, ori.y, ori.z)
        self.imu_roll = r
        self.imu_pitch = p
        self.imu_yaw = y

        if not self.has_imu:
            self.fused_yaw = y
            self.prev_imu_yaw = y
            self.has_imu = True
            return

        # Track relative heading changes from 100 Hz IMU
        dy = normalize_angle(y - self.prev_imu_yaw)
        self.fused_yaw = normalize_angle(self.fused_yaw + dy)
        self.prev_imu_yaw = y

    def visual_cb(self, msg: Odometry):
        ori = msg.pose.pose.orientation
        pos = msg.pose.pose.position

        if ori.w == 0.0 and ori.x == 0.0 and ori.y == 0.0 and ori.z == 0.0:
            return

        # Camera optical frame -> pool odom frame (x_odom = x_c, y_odom = -y_c, z=0.0)
        raw_x = pos.x
        raw_y = -pos.y

        # Transform quaternion with Rx(pi): (x_q, y_q, z_q, w_q) = (qw, -qz, qy, -qx)
        x_q = ori.w
        y_q = -ori.z
        z_q = ori.y
        w_q = -ori.x

        norm = math.sqrt(x_q*x_q + y_q*y_q + z_q*z_q + w_q*w_q)
        if norm > 1e-12:
            x_q /= norm; y_q /= norm; z_q /= norm; w_q /= norm
        else:
            w_q, x_q, y_q, z_q = 1.0, 0.0, 0.0, 0.0

        # Gating: Reject inverted Z (normal must point upwards into ceiling)
        z_dot = 1.0 - 2.0 * (x_q * x_q + y_q * y_q)
        if z_dot <= 0.0:
            return

        # Extract camera yaw
        siny_cosp = 2.0 * (w_q * z_q + x_q * y_q)
        cosy_cosp = 1.0 - 2.0 * (y_q * y_q + z_q * z_q)
        cam_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Position update
        if not self.has_visual:
            self.x = raw_x
            self.y = raw_y
            self.has_visual = True
        else:
            self.x = (1.0 - self.alpha_pos) * self.x + self.alpha_pos * raw_x
            self.y = (1.0 - self.alpha_pos) * self.y + self.alpha_pos * raw_y

        # Camera corrects IMU yaw drift
        if not self.has_imu:
            self.fused_yaw = cam_yaw
        else:
            yaw_err = normalize_angle(cam_yaw - self.fused_yaw)
            self.fused_yaw = normalize_angle(self.fused_yaw + self.yaw_corr_rate * yaw_err)

    def timer_tick(self):
        stamp = self.get_clock().now().to_msg()

        # Orientation:
        # - Roll & Pitch directly from IMU (reliable gravity vector)
        # - Yaw from camera-corrected IMU
        qw, qx, qy, qz = euler_to_quat(self.imu_roll, self.imu_pitch, self.fused_yaw)

        # Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0  # STRICTLY 0.0 ALWAYS

        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz

        self.odom_pub.publish(odom_msg)

        # Broadcast TF: odom -> usv5_odom
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.frame_id
        tf_msg.child_frame_id = self.child_frame_id
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.w = qw
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DirectUsvFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
