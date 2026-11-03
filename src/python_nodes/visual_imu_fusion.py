#!/usr/bin/env python3
"""
visual_imu_fusion.py
Node 3 of localization pipeline:
Processes camera visual odometry (/usv5/visual_odom):
1. Rigid Gating:
   - Discards detections where Z is pointing down/inverted.
   - Discards detections with tilt/inclination > 30 degrees away from vertical (Z up).
2. Complete Orientation & Position Filtering:
   - Full 3D Orientation SLERP filter (spherical linear interpolation) to smooth out
     rapid jitter and jumps in full 3D quaternion space.
   - 3D Position Low-pass EMA filter (with Z anchored on water surface).
3. Broadcasts:
   - TF: odom -> usv5 (Full filtered visual pose ground truth)
   - TF: odom -> usv5_imu (Pure IMU orientation at pool origin)
4. Publishes /usv5/odom in 'odom' frame.
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
      z_odom = 0.0
      q_odom = q_x_pi * q_cam = (-qx_c, qw_c, -qz_c, qy_c) in (x,y,z,w)
    """
    x_o = x_c
    y_o = -y_c
    z_o = 0.0

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


def quaternion_slerp(q0, q1, t):
    """
    Spherical Linear Interpolation (SLERP) between two unit quaternions (w, x, y, z).
    t: interpolation factor in [0, 1].
    """
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1

    # Compute dot product
    dot = w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1

    # If dot is negative, flip one quaternion to take the shortest path on the 4D sphere
    if dot < 0.0:
        w1, x1, y1, z1 = -w1, -x1, -y1, -z1
        dot = -dot

    # Clamp dot product to prevent math domain error
    dot = min(1.0, max(-1.0, dot))

    # If quaternions are very close, use linear interpolation to avoid division by zero
    if dot > 0.9995:
        w = w0 + t * (w1 - w0)
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        z = z0 + t * (z1 - z0)
    else:
        theta_0 = math.acos(dot)
        theta = theta_0 * t
        sin_theta = math.sin(theta)
        sin_theta_0 = math.sin(theta_0)

        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        w = s0 * w0 + s1 * w1
        x = s0 * x0 + s1 * x1
        y = s0 * y0 + s1 * y1
        z = s0 * z0 + s1 * z1

    # Normalize result
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm > 1e-12:
        return (w / norm, x / norm, y / norm, z / norm)
    return (1.0, 0.0, 0.0, 0.0)


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
        self.declare_parameter('pos_timeout_sec', 1.0)

        # Filtering parameters
        self.declare_parameter('alpha_pos', 0.35)    # Position smoothing factor
        self.declare_parameter('alpha_ori', 0.25)    # Full 3D orientation SLERP factor (0.0=frozen, 1.0=raw)
        self.declare_parameter('max_tilt_deg', 30.0) # Maximum allowed tilt away from vertical Z

        self.visual_odom_topic = self.get_parameter('visual_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.child_frame_id = self.get_parameter('child_frame_id').value
        self.water_depth = float(self.get_parameter('water_depth').value)
        self.rate = float(self.get_parameter('publish_rate').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.pos_timeout = float(self.get_parameter('pos_timeout_sec').value)
        self.alpha_pos = float(self.get_parameter('alpha_pos').value)
        self.alpha_ori = float(self.get_parameter('alpha_ori').value)
        self.max_tilt_deg = float(self.get_parameter('max_tilt_deg').value)

        # Minimum required dot product with vertical Z axis: cos(max_tilt)
        self.min_z_dot = math.cos(math.radians(self.max_tilt_deg))

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Odometry, self.visual_odom_topic, self.visual_odom_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 10)

        # Filtered State
        self.filtered_pos = None      # (x, y, z)
        self.filtered_ori = None      # (qw, qx, qy, qz)
        self.last_visual_time = None

        self.prev_pos = None
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Pure IMU Pose (at origin (0, 0, 0))
        self.latest_imu_ori = None
        self.latest_raw_imu_yaw = None
        self.latest_imu_time = None
        self.yaw_offset = None
        self.angular_vel_z = 0.0

        timer_period = 1.0 / self.rate
        self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"Visual Localization Node initialized:\n"
            f"  Visual Odometry Input: {self.visual_odom_topic}\n"
            f"  IMU Reference Input: {self.imu_topic}\n"
            f"  Gating Threshold: Max Tilt = {self.max_tilt_deg} deg (min Z-dot = {self.min_z_dot:.3f})\n"
            f"  Filters: Position Alpha = {self.alpha_pos}, Full Orientation SLERP Alpha = {self.alpha_ori}\n"
            f"  TF Frames Broadcasted:\n"
            f"    - {self.frame_id} -> {self.child_frame_id} (Full Filtered Pose)\n"
            f"    - {self.frame_id} -> {self.child_frame_id}_imu (Pure IMU orientation at origin)\n"
            f"  Pool Center Depth (Z): {self.water_depth} m"
        )

    def imu_cb(self, msg: Imu):
        """Stores IMU orientation for diagnostic TF."""
        now_sec = time.time()
        ori = msg.orientation
        self.angular_vel_z = msg.angular_velocity.z

        if not (ori.w == 0.0 and ori.x == 0.0 and ori.y == 0.0 and ori.z == 0.0):
            raw_qw, raw_qx, raw_qy, raw_qz = ori.w, ori.x, ori.y, ori.z
            self.latest_raw_imu_yaw = quaternion_to_yaw_imu(raw_qw, raw_qx, raw_qy, raw_qz)
            self.latest_imu_ori = (raw_qw, raw_qx, raw_qy, raw_qz)
            self.latest_imu_time = now_sec

    def visual_odom_cb(self, msg: Odometry):
        """
        Receives pure camera visual odometry:
        1. Rigid Tilt & Inverted Z Gate:
           In pool 'odom' frame, normal tag vector pointing UP is +Z=(0,0,1).
           The tag normal in pool frame is R_tag * (0,0,1)^T:
           nz = 1.0 - 2.0 * (qx^2 + qy^2).
           - If nz <= 0: Z is inverted (pointing down into pool) -> DISCARD.
           - If nz < cos(30 deg) (~0.866): tag tilted more than 30 deg -> DISCARD.
        2. Filter:
           - 3D Position low-pass EMA.
           - Full 3D Orientation SLERP interpolation.
        """
        now_sec = time.time()
        ori_tag = msg.pose.pose.orientation
        pos = msg.pose.pose.position

        # Ignore empty/uninitialized quaternions
        if ori_tag.w == 0.0 and ori_tag.x == 0.0 and ori_tag.y == 0.0 and ori_tag.z == 0.0:
            return

        # 1. Transform raw measurement into pool surface frame 'odom' (+Z points up)
        x_c, y_c, z_c = pos.x, pos.y, self.water_depth
        qw_c, qx_c, qy_c, qz_c = ori_tag.w, ori_tag.x, ori_tag.y, ori_tag.z

        raw_x_o, raw_y_o, _, qw_o, qx_o, qy_o, qz_o = transform_cam_pose_to_pool(
            x_c, y_c, z_c, qw_c, qx_c, qy_c, qz_c, self.water_depth
        )

        # 2. Check Z-axis orientation in pool frame:
        # The local +Z axis of the tag rotated into 'odom' frame has Z-component:
        # z_dot = R33 = 1.0 - 2.0 * (qx_o^2 + qy_o^2)
        z_dot = 1.0 - 2.0 * (qx_o * qx_o + qy_o * qy_o)

        # Gate 1: Z pointing downwards
        if z_dot <= 0.0:
            self.get_logger().warn(
                "Visual detection discarded: Z axis is inverted (pointing downwards).",
                throttle_duration_sec=2.0
            )
            return

        # Gate 2: Tilt angle exceeds threshold (> 30 deg)
        if z_dot < self.min_z_dot:
            tilt_deg = math.degrees(math.acos(min(1.0, max(-1.0, z_dot))))
            self.get_logger().warn(
                f"Visual detection discarded: excessive tilt ({tilt_deg:.1f} deg > {self.max_tilt_deg:.1f} deg).",
                throttle_duration_sec=2.0
            )
            return

        # 3. Measurement Passed Gating -> Apply Filters
        raw_pos = (raw_x_o, raw_y_o, 0.0)
        raw_ori = (qw_o, qx_o, qy_o, qz_o)

        if self.filtered_pos is None or self.filtered_ori is None:
            # Initialize with first valid measurement
            self.filtered_pos = raw_pos
            self.filtered_ori = raw_ori
        else:
            # Position EMA filter
            fx = (1.0 - self.alpha_pos) * self.filtered_pos[0] + self.alpha_pos * raw_pos[0]
            fy = (1.0 - self.alpha_pos) * self.filtered_pos[1] + self.alpha_pos * raw_pos[1]
            self.filtered_pos = (fx, fy, 0.0)

            # Complete 3D Orientation SLERP filter
            self.filtered_ori = quaternion_slerp(self.filtered_ori, raw_ori, self.alpha_ori)

        # Calculate linear velocities
        if self.prev_pos is not None:
            px, py, _, pt = self.prev_pos
            dt = now_sec - pt
            if dt > 0.001:
                self.vx = (self.filtered_pos[0] - px) / dt
                self.vy = (self.filtered_pos[1] - py) / dt
                self.vz = 0.0

        self.prev_pos = (self.filtered_pos[0], self.filtered_pos[1], 0.0, now_sec)
        self.last_visual_time = now_sec

        # Keep IMU yaw offset updated for origin reference
        if self.latest_raw_imu_yaw is not None:
            qw_f, qx_f, qy_f, qz_f = self.filtered_ori
            current_yaw = math.atan2(2.0 * (qw_f * qz_f + qx_f * qy_f),
                                     1.0 - 2.0 * (qy_f * qy_f + qz_f * qz_f))
            current_diff = normalize_angle(current_yaw - self.latest_raw_imu_yaw)
            if self.yaw_offset is None:
                self.yaw_offset = current_diff
            else:
                self.yaw_offset = normalize_angle(self.yaw_offset + 0.01 * normalize_angle(current_diff - self.yaw_offset))

    def timer_callback(self):
        """Publishes the filtered visual pose and TFs."""
        now_sec = time.time()
        stamp = self.get_clock().now().to_msg()

        # -------------------------------------------------------------
        # TF 1: Filtered Visual Pose (odom -> usv5)
        # Position X, Y on water surface, full 3D filtered orientation
        # -------------------------------------------------------------
        if self.filtered_pos is not None and self.filtered_ori is not None and self.last_visual_time is not None:
            if (now_sec - self.last_visual_time) <= self.pos_timeout:
                x, y, z = self.filtered_pos
                qw, qx, qy, qz = self.filtered_ori

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

                # Strong confidence covariance for camera ground truth:
                # X, Y: 0.001 (1mm stddev)
                # Yaw: 0.005 (0.07 rad / 4 deg stddev)
                odom_msg.pose.covariance = [
                    1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
                    0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
                    0.0,  0.0,  1e-6, 0.0,  0.0,  0.0,
                    0.0,  0.0,  0.0,  1e-6, 0.0,  0.0,
                    0.0,  0.0,  0.0,  0.0,  1e-6, 0.0,
                    0.0,  0.0,  0.0,  0.0,  0.0,  5e-3
                ]

                odom_msg.twist.twist.linear.x = self.vx
                odom_msg.twist.twist.linear.y = self.vy
                odom_msg.twist.twist.linear.z = 0.0
                odom_msg.twist.twist.angular.z = self.angular_vel_z

                odom_msg.twist.covariance = [
                    0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.02
                ]

                self.odom_pub.publish(odom_msg)

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
