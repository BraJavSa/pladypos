#!/usr/bin/env python3
"""
inverse_dynamics_controller.py
Real-time 30 Hz 3-DOF Fossen Inverse Dynamics (Computed Torque) Controller
for the PlaDyPos USV (usv5), tracking a Figure-8 trajectory centered in the
pool workspace (70% of the navigable pool area, centered at (X_c, Y_c), NOT at (0,0) of odom).

Publishes:
  - /usv5/pwm_out        (std_msgs/Float32MultiArray): 4 thruster commands in [-1.0, 1.0]
  - /usv5/reference_path (nav_msgs/Path): Full Figure-8 reference path for RViz
  - /usv5/executed_path  (nav_msgs/Path): Executed USV path for RViz
  - /pool_boundary       (visualization_msgs/Marker): Figure-8 reference curve,
                         start point, moving target pose arrow, and USV trail in RViz
"""

import csv
import math
import os
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, ColorRGBA, Float32MultiArray
from visualization_msgs.msg import Marker


# ==============================================================================
# IDENTIFIED 9-PARAMETER WHITE-BOX FOSSEN MODEL & THRUSTER ALLOCATION
# ==============================================================================
DEFAULT_FOSSEN_MODEL = {
    "m11": 219.872606,
    "m22": 230.539377,
    "m33": 9.584423,
    "Xu": 136.221265,
    "Xuu": 77.904259,
    "Yv": 109.897561,
    "Yvv": 52.401440,
    "Nr": 14.456386,
    "Nrr": 2.194899,
}

D_ARM = 0.65 / math.sqrt(2.0)             # 0.459619 m
COS45 = math.cos(math.radians(45.0))      # 0.707107
SIN45 = math.sin(math.radians(45.0))      # 0.707107

# Order: [M1: FR, M2: FL, M3: BR, M4: BL]
PHI_ALLOC = np.array([
    [ COS45,  COS45, -COS45, -COS45],
    [ SIN45, -SIN45,  SIN45, -SIN45],
    [ D_ARM, -D_ARM, -D_ARM,  D_ARM]
], dtype=float)
PHI_PINV = np.linalg.pinv(PHI_ALLOC)

RICHARDS_PARAMS = {
    'A_pos': 0.000001, 'K_pos': 40.0209,
    'B_pos': 2.6249,   'v_pos': 0.1615,
    'C_pos': 0.9432,   'M_pos': 0.00001,
    'maxForceFwd': 36.3827,
    'A_neg': -31.4990, 'K_neg': -0.00001,
    'B_neg': 3.6986,   'v_neg': 0.3264,
    'C_neg': 0.9713,   'M_neg': -1.0000,
    'maxForceRev': -28.4393,
    'deadzone': 0.01
}


def wrap_to_pi(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return math.cos(half), 0.0, 0.0, math.sin(half)


def thrust_to_cmd_richards(T_des: np.ndarray) -> np.ndarray:
    """Inverts the Richards logistic curve to map desired thrust [N] to command in [-1, 1]."""
    p = RICHARDS_PARAMS
    T_arr = np.asarray(T_des, dtype=float)
    cmd_out = np.zeros_like(T_arr, dtype=float)

    for i, f_val in enumerate(T_arr.flat):
        if abs(f_val) < 0.15:
            cmd_out.flat[i] = 0.0
        elif f_val > 0.0:
            f_c = float(np.clip(f_val, 0.05, p['maxForceFwd'] - 1e-3))
            ratio = (p['K_pos'] - p['A_pos']) / (f_c - p['A_pos'])
            inner = max(1e-9, (ratio ** p['v_pos']) - p['C_pos'])
            c = p['M_pos'] - math.log(inner) / p['B_pos']
            cmd_out.flat[i] = float(np.clip(c, p['deadzone'], 1.0))
        else:
            f_c = float(np.clip(f_val, p['maxForceRev'] + 1e-3, -0.05))
            ratio = (p['K_neg'] - p['A_neg']) / (f_c - p['A_neg'])
            inner = max(1e-9, (ratio ** p['v_neg']) - p['C_neg'])
            c = p['M_neg'] - math.log(inner) / p['B_neg']
            cmd_out.flat[i] = float(np.clip(c, -1.0, -p['deadzone']))

    return cmd_out


class InverseDynamicsControllerNode(Node):
    def __init__(self):
        super().__init__('inverse_dynamics_controller')

        # Pool-centered Figure-8 parameters (70% of experimental pool bounds)
        # Pool X: [-3.0859, 1.9433] -> Center X_c = -0.5713 m, Span_X = 5.0292 m -> A_x(70%) = 1.7602 m
        # Pool Y: [-3.6876, 1.1414] -> Center Y_c = -1.2731 m, Span_Y = 4.8290 m -> A_y(70%) = 1.6902 m
        self.declare_parameter('center_x', -0.5713)
        self.declare_parameter('center_y', -1.2731)
        self.declare_parameter('span_x', 5.0292)
        self.declare_parameter('span_y', 4.8290)
        self.declare_parameter('scale_factor', 0.70)
        self.declare_parameter('period_s', 55.0)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('soft_start_s', 3.0)
        self.declare_parameter('kp_xy', 2.25)
        self.declare_parameter('kd_xy', 3.00)
        self.declare_parameter('kp_yaw', 4.84)
        self.declare_parameter('kd_yaw', 4.40)
        self.declare_parameter('odom_topic', '/usv5/odometry/filtered')
        self.declare_parameter('visual_odom_topic', '/usv5/visual_odom')
        self.declare_parameter('imu_topic', '/usv5/imu/data')
        self.declare_parameter('pwm_topic', '/usv5/pwm_out')
        self.declare_parameter('marker_topic', '/pool_boundary')
        self.declare_parameter('frame_id', 'odom')

        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.span_x = float(self.get_parameter('span_x').value)
        self.span_y = float(self.get_parameter('span_y').value)
        self.scale_factor = float(self.get_parameter('scale_factor').value)
        self.ax_8 = 0.5 * self.span_x * self.scale_factor
        self.ay_8 = 0.5 * self.span_y * self.scale_factor
        self.period_s = float(self.get_parameter('period_s').value)
        self.omega = 2.0 * math.pi / self.period_s
        self.rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.dt = 1.0 / self.rate_hz
        self.soft_start_s = float(self.get_parameter('soft_start_s').value)

        kp_xy = float(self.get_parameter('kp_xy').value)
        kd_xy = float(self.get_parameter('kd_xy').value)
        kp_yaw = float(self.get_parameter('kp_yaw').value)
        kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.Kp = np.diag([kp_xy, kp_xy, kp_yaw])
        self.Kd = np.diag([kd_xy, kd_xy, kd_yaw])

        self.odom_topic = self.get_parameter('odom_topic').value
        self.visual_odom_topic = self.get_parameter('visual_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.pwm_topic = self.get_parameter('pwm_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.model = DEFAULT_FOSSEN_MODEL.copy()
        self._load_motor_mapping()

        # Publishers
        self.pwm_pub = self.create_publisher(Float32MultiArray, self.pwm_topic, 10)
        self.auto_pub = self.create_publisher(Bool, '/usv5/autonomous_active', 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.ref_path_pub = self.create_publisher(NavPath, '/usv5/reference_path', 10)
        self.exec_path_pub = self.create_publisher(NavPath, '/usv5/executed_path', 10)

        # Subscribers
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(Odometry, self.visual_odom_topic, self.visual_odom_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 20)

        # Real-time state variables
        self.has_odom = False
        self.has_visual_vel = False
        self.has_imu_r = False
        self.x = self.center_x
        self.y = self.center_y
        self.yaw = 0.0
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.prev_odom_time = None
        self.vx_odom = 0.0
        self.vy_odom = 0.0
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0

        self.start_wall_time = None
        self.log_rows = []
        self.exec_poses = []
        self.exec_points = []

        # Precompute full Figure-8 path for RViz visualization
        self.ref_path_msg, self.ref_marker_points = self._build_reference_path_messages()

        # 30 Hz Control & Visualization Timer
        self.create_timer(self.dt, self.control_loop_tick)
        # 1 Hz Static Reference Marker Publisher (so RViz always shows the Figure-8)
        self.create_timer(1.0, self.publish_reference_visuals)

        self.get_logger().info(
            f"3-DOF Fossen Inverse Dynamics Controller initialized at {self.rate_hz:.0f} Hz:\n"
            f"  Pool Center (X_c, Y_c) : ({self.center_x:.4f}, {self.center_y:.4f}) m\n"
            f"  70% Figure-8 Semi-Axes : A_x = {self.ax_8:.4f} m, A_y = {self.ay_8:.4f} m\n"
            f"  Figure-8 Period        : {self.period_s:.1f} s\n"
            f"  PWM Output Topic       : {self.pwm_topic}\n"
            f"  RViz Topics            : {self.marker_topic}, /usv5/reference_path, /usv5/executed_path"
        )

    def _load_motor_mapping(self):
        self.motor_map = {
            1: "Front Right (FR)",
            2: "Front Left (FL)",
            3: "Back Right (BR)",
            4: "Back Left (BL)"
        }
        try:
            pkg_share = get_package_share_directory('pladypos')
            cfg_path = os.path.join(pkg_share, 'config', 'motor_mapping.yaml')
            if os.path.exists(cfg_path):
                with open(cfg_path, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'motor_mapping' in data:
                        self.motor_map = data['motor_mapping']
        except Exception:
            pass

    def evaluate_figure8(self, t_sec: float):
        """Evaluates [xd, yd, psid], [dxd, dyd, dpsid], [ddxd, ddyd, ddpsid] at time t_sec."""
        w = self.omega
        Ax, Ay = self.ax_8, self.ay_8
        xc, yc = self.center_x, self.center_y

        xd = xc + Ax * math.sin(w * t_sec)
        yd = yc + Ay * math.sin(2.0 * w * t_sec)

        dxd = Ax * w * math.cos(w * t_sec)
        dyd = 2.0 * Ay * w * math.cos(2.0 * w * t_sec)

        ddxd = -Ax * (w ** 2) * math.sin(w * t_sec)
        ddyd = -4.0 * Ay * (w ** 2) * math.sin(2.0 * w * t_sec)

        dddxd = -Ax * (w ** 3) * math.cos(w * t_sec)
        dddyd = -8.0 * Ay * (w ** 3) * math.cos(2.0 * w * t_sec)

        psid = math.atan2(dyd, dxd)
        v2 = dxd * dxd + dyd * dyd + 1e-12
        dpsid = (dxd * ddyd - dyd * ddxd) / v2
        dv2_dt = 2.0 * (dxd * ddxd + dyd * ddyd)
        ddpsid = ((dxd * dddyd - dyd * dddxd) * v2 - (dxd * ddyd - dyd * ddxd) * dv2_dt) / (v2 * v2)

        eta_d = np.array([xd, yd, psid], dtype=float)
        deta_d = np.array([dxd, dyd, dpsid], dtype=float)
        ddeta_d = np.array([ddxd, ddyd, ddpsid], dtype=float)
        return eta_d, deta_d, ddeta_d

    def _build_reference_path_messages(self):
        stamp = self.get_clock().now().to_msg()
        path_msg = NavPath()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.frame_id

        pts = []
        n_pts = 360
        for i in range(n_pts + 1):
            t_i = (i / n_pts) * self.period_s
            eta_i, _, _ = self.evaluate_figure8(t_i)
            qw, qx, qy, qz = yaw_to_quat(eta_i[2])

            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = float(eta_i[0])
            ps.pose.position.y = float(eta_i[1])
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = qw
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            path_msg.poses.append(ps)

            pt = Point()
            pt.x = float(eta_i[0])
            pt.y = float(eta_i[1])
            pt.z = 0.02
            pts.append(pt)

        return path_msg, pts

    def publish_reference_visuals(self):
        stamp = self.get_clock().now().to_msg()
        self.ref_path_msg.header.stamp = stamp
        self.ref_path_pub.publish(self.ref_path_msg)

        # 1. Full Figure-8 Reference Curve Marker on /pool_boundary
        fig8_marker = Marker()
        fig8_marker.header.stamp = stamp
        fig8_marker.header.frame_id = self.frame_id
        fig8_marker.ns = "figure8_reference"
        fig8_marker.id = 10
        fig8_marker.type = Marker.LINE_STRIP
        fig8_marker.action = Marker.ADD
        fig8_marker.scale.x = 0.045
        fig8_marker.color = ColorRGBA(r=0.95, g=0.22, b=0.22, a=0.95)  # Crimson Red
        fig8_marker.pose.orientation.w = 1.0
        fig8_marker.points = self.ref_marker_points
        self.marker_pub.publish(fig8_marker)

        # 2. Start / Pool Center Point Marker (Green Sphere at (X_c, Y_c))
        start_marker = Marker()
        start_marker.header.stamp = stamp
        start_marker.header.frame_id = self.frame_id
        start_marker.ns = "figure8_start"
        start_marker.id = 11
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.center_x
        start_marker.pose.position.y = self.center_y
        start_marker.pose.position.z = 0.05
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.18
        start_marker.scale.y = 0.18
        start_marker.scale.z = 0.18
        start_marker.color = ColorRGBA(r=0.06, g=0.72, b=0.51, a=0.95)  # Emerald Green
        self.marker_pub.publish(start_marker)

    def imu_cb(self, msg: Imu):
        self.r = float(msg.angular_velocity.z)
        self.has_imu_r = True

    def visual_odom_cb(self, msg: Odometry):
        # Transform camera optical velocity to pool odom frame (vx_odom = vx_cam, vy_odom = -vy_cam)
        self.vx_odom = float(msg.twist.twist.linear.x)
        self.vy_odom = -float(msg.twist.twist.linear.y)
        self.has_visual_vel = True

    def odom_cb(self, msg: Odometry):
        now_s = time.monotonic()
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        x_new = float(pos.x)
        y_new = float(pos.y)
        yaw_new = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)

        if self.prev_odom_time is not None:
            dt_odom = max(1e-3, now_s - self.prev_odom_time)
            if not self.has_visual_vel:
                vx_raw = (x_new - self.prev_x) / dt_odom
                vy_raw = (y_new - self.prev_y) / dt_odom
                self.vx_odom = 0.6 * self.vx_odom + 0.4 * vx_raw
                self.vy_odom = 0.6 * self.vy_odom + 0.4 * vy_raw
            if not self.has_imu_r:
                dyaw = wrap_to_pi(yaw_new - self.prev_yaw)
                self.r = 0.6 * self.r + 0.4 * (dyaw / dt_odom)

        self.x = x_new
        self.y = y_new
        self.yaw = yaw_new
        self.prev_x = x_new
        self.prev_y = y_new
        self.prev_yaw = yaw_new
        self.prev_odom_time = now_s

        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        self.u = self.vx_odom * cos_y + self.vy_odom * sin_y
        self.v = -self.vx_odom * sin_y + self.vy_odom * cos_y
        self.has_odom = True

    def control_loop_tick(self):
        if not self.has_odom:
            return

        now_s = time.monotonic()
        if self.start_wall_time is None:
            self.start_wall_time = now_s
            self.get_logger().info("First odometry received! Starting 30 Hz Figure-8 Inverse Dynamics tracking.")

        t_elapsed = now_s - self.start_wall_time

        # Smooth soft-start ramp over self.soft_start_s seconds
        if self.soft_start_s > 0.0 and t_elapsed < self.soft_start_s:
            s_ramp = 0.5 * (1.0 - math.cos(math.pi * t_elapsed / self.soft_start_s))
        else:
            s_ramp = 1.0

        eta_d, deta_d, ddeta_d = self.evaluate_figure8(t_elapsed)
        deta_d = s_ramp * deta_d
        ddeta_d = (s_ramp ** 2) * ddeta_d

        # Current state vectors
        eta = np.array([self.x, self.y, self.yaw], dtype=float)
        nu = np.array([self.u, self.v, self.r], dtype=float)

        c_psi = math.cos(self.yaw)
        s_psi = math.sin(self.yaw)
        J = np.array([
            [c_psi, -s_psi, 0.0],
            [s_psi,  c_psi, 0.0],
            [0.0,    0.0,   1.0]
        ], dtype=float)
        deta = J @ nu

        # Pose tracking error (with soft-bounded position error so initial offset doesn't saturate)
        ex = eta_d[0] - eta[0]
        ey = eta_d[1] - eta[1]
        epsi = wrap_to_pi(eta_d[2] - eta[2])
        e_eta = np.array([ex, ey, epsi], dtype=float)
        de_eta = deta_d - deta

        # 1. Outer kinematic PD + feedforward acceleration in inertial frame
        a_eta = ddeta_d + (self.Kd @ de_eta) + (self.Kp @ e_eta)

        # 2. Exact body-frame acceleration command: a_nu = J^T * a_eta - S(r)*nu
        s_nu = np.array([-self.v * self.r, self.u * self.r, 0.0], dtype=float)
        a_nu = (J.T @ a_eta) - s_nu

        # 3. 3-DOF Fossen Inverse Dynamics Law: tau = M * a_nu + C(nu)*nu + D(nu)*nu
        m = self.model
        M_diag = np.array([m["m11"], m["m22"], m["m33"]], dtype=float)
        C_nu = np.array([
            -m["m22"] * self.v * self.r,
             m["m11"] * self.u * self.r,
            (m["m22"] - m["m11"]) * self.u * self.v
        ], dtype=float)
        D_nu = np.array([
            (m["Xu"] + m["Xuu"] * abs(self.u)) * self.u,
            (m["Yv"] + m["Yvv"] * abs(self.v)) * self.v,
            (m["Nr"] + m["Nrr"] * abs(self.r)) * self.r
        ], dtype=float)

        tau_cmd = s_ramp * (M_diag * a_nu + C_nu + D_nu)

        # 4. Thruster Allocation & Richards Inverse Mapping -> [-1.0, 1.0]
        f_motors = PHI_PINV @ tau_cmd
        cmd_norm = thrust_to_cmd_richards(f_motors)
        u_FR, u_FL, u_BR, u_BL = float(cmd_norm[0]), float(cmd_norm[1]), float(cmd_norm[2]), float(cmd_norm[3])

        # Map to physical Arduino channels via motor_mapping.yaml
        out_data = [0.0, 0.0, 0.0, 0.0]
        for channel, position in self.motor_map.items():
            ch_idx = int(channel) - 1
            if 0 <= ch_idx < 4:
                if "Front Right" in position or "FR" in position:
                    out_data[ch_idx] = u_FR
                elif "Front Left" in position or "FL" in position:
                    out_data[ch_idx] = u_FL
                elif "Back Right" in position or "BR" in position:
                    out_data[ch_idx] = u_BR
                elif "Back Left" in position or "BL" in position:
                    out_data[ch_idx] = u_BL

        auto_msg = Bool()
        auto_msg.data = True
        self.auto_pub.publish(auto_msg)

        pwm_msg = Float32MultiArray()
        pwm_msg.data = out_data
        self.pwm_pub.publish(pwm_msg)

        # Publish real-time target carrot and executed trajectory to RViz
        self._publish_realtime_rviz(eta_d)

        # Log sample
        c_d = math.cos(eta_d[2])
        s_d = math.sin(eta_d[2])
        u_ref = c_d * deta_d[0] + s_d * deta_d[1]
        v_ref = -s_d * deta_d[0] + c_d * deta_d[1]
        r_ref = deta_d[2]
        self.log_rows.append([
            f"{t_elapsed:.4f}",
            f"{eta_d[0]:.5f}", f"{eta_d[1]:.5f}", f"{eta_d[2]:.5f}",
            f"{self.x:.5f}", f"{self.y:.5f}", f"{self.yaw:.5f}",
            f"{u_ref:.5f}", f"{v_ref:.5f}", f"{r_ref:.5f}",
            f"{self.u:.5f}", f"{self.v:.5f}", f"{self.r:.5f}",
            f"{tau_cmd[0]:.4f}", f"{tau_cmd[1]:.4f}", f"{tau_cmd[2]:.4f}",
            f"{out_data[0]:.4f}", f"{out_data[1]:.4f}", f"{out_data[2]:.4f}", f"{out_data[3]:.4f}"
        ])

    def _publish_realtime_rviz(self, eta_d: np.ndarray):
        stamp = self.get_clock().now().to_msg()
        qw_d, qx_d, qy_d, qz_d = yaw_to_quat(float(eta_d[2]))

        # Moving Target Arrow on the Figure-8
        target_marker = Marker()
        target_marker.header.stamp = stamp
        target_marker.header.frame_id = self.frame_id
        target_marker.ns = "figure8_target"
        target_marker.id = 12
        target_marker.type = Marker.ARROW
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = float(eta_d[0])
        target_marker.pose.position.y = float(eta_d[1])
        target_marker.pose.position.z = 0.06
        target_marker.pose.orientation.w = qw_d
        target_marker.pose.orientation.x = qx_d
        target_marker.pose.orientation.y = qy_d
        target_marker.pose.orientation.z = qz_d
        target_marker.scale.x = 0.35
        target_marker.scale.y = 0.08
        target_marker.scale.z = 0.08
        target_marker.color = ColorRGBA(r=1.0, g=0.80, b=0.0, a=0.95)  # Gold Arrow
        self.marker_pub.publish(target_marker)

        # Append current boat pose to executed trail
        pt = Point()
        pt.x = float(self.x)
        pt.y = float(self.y)
        pt.z = 0.03
        self.exec_points.append(pt)
        if len(self.exec_points) > 1800:
            self.exec_points = self.exec_points[-1800:]

        trail_marker = Marker()
        trail_marker.header.stamp = stamp
        trail_marker.header.frame_id = self.frame_id
        trail_marker.ns = "usv_executed_trail"
        trail_marker.id = 13
        trail_marker.type = Marker.LINE_STRIP
        trail_marker.action = Marker.ADD
        trail_marker.scale.x = 0.035
        trail_marker.color = ColorRGBA(r=0.10, g=0.90, b=0.35, a=0.95)  # Bright Green Trail
        trail_marker.pose.orientation.w = 1.0
        trail_marker.points = self.exec_points
        self.marker_pub.publish(trail_marker)

        qw, qx, qy, qz = yaw_to_quat(self.yaw)
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(self.x)
        ps.pose.position.y = float(self.y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = qw
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        self.exec_poses.append(ps)
        if len(self.exec_poses) > 1800:
            self.exec_poses = self.exec_poses[-1800:]

        exec_path_msg = NavPath()
        exec_path_msg.header.stamp = stamp
        exec_path_msg.header.frame_id = self.frame_id
        exec_path_msg.poses = self.exec_poses
        self.exec_path_pub.publish(exec_path_msg)

    def stop_thrusters_and_save_log(self):
        # Release autonomous lock and send neutral PWM [0, 0, 0, 0] (1500 us)
        auto_msg = Bool()
        auto_msg.data = False
        stop_msg = Float32MultiArray()
        stop_msg.data = [0.0, 0.0, 0.0, 0.0]
        for _ in range(5):
            self.auto_pub.publish(auto_msg)
            self.pwm_pub.publish(stop_msg)
            time.sleep(0.02)
        self.get_logger().info("Sent neutral stop command [0.0, 0.0, 0.0, 0.0] to thrusters.")

        if self.log_rows:
            candidate_dirs = [
                Path("/home/brayan/ros2_ws/src/pladypos/experiments/processed_data"),
                Path.home() / "ros2usv_ws" / "src" / "pladypos" / "experiments" / "processed_data",
                Path.home() / "ros2_ws" / "src" / "pladypos" / "experiments" / "processed_data",
                Path.home() / "pladypos_logs",
            ]
            out_dir = next((d.parent.parent for d in candidate_dirs[:3] if d.parent.parent.exists()), None)
            if out_dir is not None:
                out_dir = out_dir / "experiments" / "processed_data"
            else:
                out_dir = candidate_dirs[-1]
            out_dir.mkdir(parents=True, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_file = out_dir / f"realtime_inverse_dynamics_{ts}.csv"
            latest_file = out_dir / "realtime_inverse_dynamics_30Hz.csv"
            header = [
                "t",
                "x_ref", "y_ref", "psi_ref",
                "x_real", "y_real", "psi_real",
                "u_ref", "v_ref", "r_ref",
                "u_real", "v_real", "r_real",
                "Tu", "Tv", "Tr",
                "cmd_m1_FR", "cmd_m2_FL", "cmd_m3_BR", "cmd_m4_BL"
            ]
            for target_path in (csv_file, latest_file):
                with open(target_path, "w", newline="", encoding="utf-8") as fp:
                    writer = csv.writer(fp)
                    writer.writerow(header)
                    writer.writerows(self.log_rows)
            self.get_logger().info(f"Saved {len(self.log_rows)} samples to {latest_file}")


def main(args=None):
    rclpy.init(args=args)
    node = InverseDynamicsControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop_thrusters_and_save_log()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

