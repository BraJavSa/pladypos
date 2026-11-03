#!/usr/bin/env python3
"""
visual_imu_fusion.py
Node 3 of localization pipeline:
Validates camera visual odometry (/usv5/visual_odom) using the onboard IMU (/usv5/imu/data).

- The camera is ground truth for BOTH position and orientation.
- The IMU is used ONLY as an approximate reference / sanity check:
  * Discards visual measurements where the Z axis is inverted (pointing into pool).
  * Discards visual measurements where orientation differs drastically from IMU (> 45 deg).
- When validated:
  * Transforms the position and orientation into the static pool surface frame (odom)
    located at the center of the pool camera image at water level (Z = 5.1m below camera),
    where +Z points UP (matching the IMU world Z and gravity convention).
  * Publishes /usv5/odom relative to 'odom' (pool center).
  * Broadcasts the dynamic TF: odom -> usv5.
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
    """Normalize angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw_camera(qw: float, qx: float, qy: float, qz: float) -> float:
    """Computes heading angle (yaw) of the tag in camera/pool horizontal plane."""
    r10 = 2.0 * (qx * qy + qw * qz)
    r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(-r10, r00)


def quaternion_to_yaw_imu(qw: float, qx: float, qy: float, qz: float) -> float:
    """Computes heading angle (yaw) from ENU IMU quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_cam_pose_to_pool(x_c: float, y_c: float, z_c: float,
                               qw_c: float, qx_c: float, qy_c: float, qz_c: float,
                               water_depth: float = 5.1):
    """
    Transforms pose from camera optical frame (X right, Y down, Z forward down)
    to static pool surface frame 'odom' (X right, Y forward/up, Z up):
    Static transform: camera -> odom is T=(0, 0, water_depth), R=Rx(pi).
    So:
      x_odom = x_c
      y_odom = -y_c
      z_odom = -(z_c - water_depth)  (~0.0 on water surface)
      q_odom = q_x_pi * q_cam = (-qx_c, qw_c, -qz_c, qy_c) in (x,y,z,w)
    """
    x_o = x_c
    y_o = -y_c
    z_o = -(z_c - water_depth)

    # Quaternion multiplication: q_p_c * q_cam_tag where q_p_c has Rx(pi) -> (x=1, y=0, z=0, w=0)
    # Using (x, y, z, w):
    # w_o = -x_c_quat
    # x_o = w_c_quat
    # y_o = -z_c_quat
    # z_o = y_c_quat
    x_q = qw_c
    y_q = -qz_c
    z_q = qy_c
    w_q = -qx_c

    norm = math.sqrt(x_q*x_q + y_q*y_q + z_q*z_q + w_q*w_q)
    if norm > 1e-12:
        x_q /= norm
        y_q /= norm
        z_q /= norm
        w_q /= norm
    else:
        w_q, x_q, y_q, z_q = 1.0, 0.0, 0.0, 0.0

    return x_o, y_o, z_o, w_q, x_q, y_q, z_q


class VisualImuFusionNode(Node):
    def __init__(self):
        super().__init__('visual_imu_fusion')

        self.declare_parameter('visual_odom_topic', '/usv5/visual_odom')
        self.declare_parameter('imu_topic', '/usv5/imu/data')
        self.declare_parameter('odom_topic', '/usv5/odom')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'usv5')
        self.declare_parameter('water_depth', 5.1)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('max_yaw_diff_deg', 45.0)
        self.declare_parameter('pos_timeout_sec', 1.0)

        self.visual_odom_topic = self.get_parameter('visual_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.water_depth = float(self.get_parameter('water_depth').value)
        self.rate = float(self.get_parameter('publish_rate').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.max_yaw_diff_rad = math.radians(float(self.get_parameter('max_yaw_diff_deg').value))
        self.pos_timeout = float(self.get_parameter('pos_timeout_sec').value)

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, self.visual_odom_topic, self.visual_odom_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 10)

        # Validated pose in 'odom' frame (Z up)
        self.latest_pos = None     # (x, y, z, timestamp)
        self.latest_ori = None     # (qw, qx, qy, qz)
        self.prev_pos = None
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # IMU orientation reference (for sanity check only)
        self.latest_raw_imu_yaw = None
        self.latest_imu_time = None
        self.yaw_offset = None
        self.angular_vel_z = 0.0

        timer_period = 1.0 / self.rate
        self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"VisualImuFusionNode (Node 3) initialized:\n"
            f"  Visual Odometry Input: {self.visual_odom_topic}\n"
            f"  IMU Reference Input: {self.imu_topic}\n"
            f"  Validated Odometry Output: {self.odom_topic} (frame: {self.frame_id} -> {self.child_frame_id})\n"
            f"  Pool Center Depth (Z): {self.water_depth} m\n"
            f"  Broadcast TF ({self.frame_id} -> {self.child_frame_id}): {self.publish_tf}"
        )

    def imu_cb(self, msg: Imu):
        """Updates IMU yaw reference for sanity checking."""
        now_sec = time.time()
        ori = msg.orientation
        self.angular_vel_z = msg.angular_velocity.z

        if not (ori.w == 0.0 and ori.x == 0.0 and ori.y == 0.0 and ori.z == 0.0):
            raw_qw, raw_qx, raw_qy, raw_qz = ori.w, ori.x, ori.y, ori.z
            self.latest_raw_imu_yaw = quaternion_to_yaw_imu(raw_qw, raw_qx, raw_qy, raw_qz)
            self.latest_imu_time = now_sec

    def visual_odom_cb(self, msg: Odometry):
        """
        Validates camera measurement against IMU.
        If valid: transforms pose into pool surface frame (odom) where Z is up.
        If invalid (Z inverted or yaw differs drastically): discards.
        """
        now_sec = time.time()
        ori_tag = msg.pose.pose.orientation

        # 1. Gate A: Reject inverted Z (in camera optical frame, normal vz >= 0 points down into pool)
        if not (ori_tag.w == 0.0 and ori_tag.x == 0.0 and ori_tag.y == 0.0 and ori_tag.z == 0.0):
            vz = 1.0 - 2.0 * (ori_tag.x ** 2 + ori_tag.y ** 2)
            if vz >= 0.0:
                self.get_logger().warn(
                    "Visual detection discarded: Z axis is inverted (pointing into pool).",
                    throttle_duration_sec=2.0
                )
                return

            yaw_tag = quaternion_to_yaw_camera(ori_tag.w, ori_tag.x, ori_tag.y, ori_tag.z)

            # 2. Gate B: Sanity check against IMU orientation (if IMU is active and fresh)
            imu_is_fresh = (
                self.latest_raw_imu_yaw is not None and
                self.latest_imu_time is not None and
                (now_sec - self.latest_imu_time) <= 1.5
            )

            if imu_is_fresh:
                diff = normalize_angle(yaw_tag - self.latest_raw_imu_yaw)
                if self.yaw_offset is None:
                    self.yaw_offset = diff
                    self.get_logger().info(
                        f"Visual-IMU reference yaw offset locked: {math.degrees(self.yaw_offset):.1f} deg"
                    )
                else:
                    discrepancy = abs(normalize_angle(diff - self.yaw_offset))
                    if discrepancy > self.max_yaw_diff_rad:
                        self.get_logger().warn(
                            f"Visual detection discarded: orientation differs from IMU "
                            f"({math.degrees(discrepancy):.1f} deg > {math.degrees(self.max_yaw_diff_rad):.1f} deg).",
                            throttle_duration_sec=2.0
                        )
                        return
                    # Slowly adapt offset to track slow IMU drift
                    self.yaw_offset = normalize_angle(self.yaw_offset + 0.02 * normalize_angle(diff - self.yaw_offset))
            else:
                self.get_logger().info(
                    "IMU unavailable or lost: validating visual pose using Z face-up check only.",
                    throttle_duration_sec=5.0
                )

        # 3. Measurement validated: Transform camera pose to pool surface frame 'odom' (Z points up)
        pos = msg.pose.pose.position
        x_c, y_c, z_c = pos.x, pos.y, self.water_depth
        qw_c, qx_c, qy_c, qz_c = ori_tag.w, ori_tag.x, ori_tag.y, ori_tag.z

        x_o, y_o, z_o, qw_o, qx_o, qy_o, qz_o = transform_cam_pose_to_pool(
            x_c, y_c, z_c, qw_c, qx_c, qy_c, qz_c, self.water_depth
        )

        if self.prev_pos is not None:
            px, py, pz, pt = self.prev_pos
            dt = now_sec - pt
            if dt > 0.001:
                self.vx = (x_o - px) / dt
                self.vy = (y_o - py) / dt
                self.vz = (z_o - pz) / dt

        self.prev_pos = (x_o, y_o, z_o, now_sec)
        self.latest_pos = (x_o, y_o, z_o, now_sec)
        self.latest_ori = (qw_o, qx_o, qy_o, qz_o)

    def timer_callback(self):
        """Publishes validated odometry and TF in 'odom' frame (Z up)."""
        if self.latest_pos is None or self.latest_ori is None:
            return

        now_sec = time.time()
        # Drop if camera detection has timed out
        if (now_sec - self.latest_pos[3]) > self.pos_timeout:
            return

        stamp = self.get_clock().now().to_msg()
        x, y, z, _ = self.latest_pos
        vx, vy, vz = self.vx, self.vy, self.vz
        qw, qx, qy, qz = self.latest_ori

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        # Position in pool odom frame (Z = 0 at pool surface, +Z up)
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z

        # Orientation in pool odom frame (yaw around +Z up)
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

        # Broadcast dynamic transform odom -> usv5
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
    node = VisualImuFusionNode()
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
