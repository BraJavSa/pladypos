#!/usr/bin/env python3
"""
inverse_dynamics_controller.py
==============================
Real-Time 30 Hz 3-DOF Fossen Inverse Dynamics Trajectory Tracking Controller
for the PlaDyPos USV tracking a centered 70% Figure-8 inside the pool.

Key Features:
1. Starts at the closest point on the Figure-8 to the boat's initial position (x0, y0)
   and smoothly blends the initial heading psi0 into the Figure-8 tangent so there is
   no abrupt initial rotation step.
2. Position-Prioritized & Symmetric Thruster Allocation:
   - Bounds thruster forces symmetrically within [-28.0, +28.0] N (inside the reverse
     saturation limit of -28.44 N) so yaw torque NEVER creates parasitic (x, y) drift
     from forward/reverse thruster asymmetry.
   - Prioritizes (tau_u, tau_v) position holding over yaw torque tau_r.
3. Ultra-Lightweight RViz Visualization:
   - Publishes ONLY a single moving sphere marker (figure8_target, id=12) at 15 Hz on
     /pool_boundary (< 100 bytes/msg), with zero Path arrays or static duplicate markers.
"""

import csv
import math
import os
import time
import yaml
from datetime import datetime
from pathlib import Path
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Bool, ColorRGBA
from visualization_msgs.msg import Marker


# ==============================================================================
# 1. IDENTIFIED 3-DOF FOSSEN PARAMETERS & THRUSTER ALLOCATION
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

# Geometric X-allocation matrix for PlaDyPos (L = 0.65 m, 45 deg azimuths)
# Order of columns: [Front-Right (FR), Front-Left (FL), Back-Right (BR), Back-Left (BL)]
D_ARM = 0.65 / math.sqrt(2.0)  # 0.459619 m
COS45 = math.cos(math.radians(45.0))
SIN45 = math.sin(math.radians(45.0))

PHI_ALLOC = np.array([
    [ COS45,  COS45, -COS45, -COS45],
    [ SIN45, -SIN45,  SIN45, -SIN45],
    [ D_ARM, -D_ARM, -D_ARM,  D_ARM]
], dtype=float)
PHI_PINV = np.linalg.pinv(PHI_ALLOC)

# Identified Asymmetric Richards Thruster Curve Parameters
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

# Symmetric per-thruster limit [N] so reverse saturation (-28.44 N) never causes parasitic XY drift
F_SYM_MAX = 28.0


def thrust_to_cmd_richards(T_des: np.ndarray) -> np.ndarray:
    """Analytical inverse of the asymmetric Richards thrust model: Newtons -> cmd in [-1, 1]."""
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


def allocate_priority_xy(tau_u: float, tau_v: float, tau_r: float) -> np.ndarray:
    """
    Allocates generalized forces [tau_u, tau_v, tau_r] across the 4 X-thrusters
    with strict priority for (x, y) translation over yaw rotation, bounded symmetrically
    by F_SYM_MAX = 28.0 N so yaw rotation NEVER induces parasitic (x, y) drift.
    """
    f_xy = PHI_PINV @ np.array([tau_u, tau_v, 0.0], dtype=float)
    max_xy = float(np.max(np.abs(f_xy)))
    # Reserve at least 4 N of symmetric headroom when possible, capped at F_SYM_MAX
    xy_limit = 24.0
    if max_xy > xy_limit:
        f_xy = f_xy * (xy_limit / max_xy)

    f_r = PHI_PINV @ np.array([0.0, 0.0, tau_r], dtype=float)
    # Find largest scale alpha in [0, 1] such that |f_xy_i + alpha * f_r_i| <= F_SYM_MAX for all i
    alpha = 1.0
    for i in range(4):
        df = abs(float(f_r[i]))
        if df > 1e-6:
            headroom = max(0.0, F_SYM_MAX - abs(float(f_xy[i])))
            alpha = min(alpha, headroom / df)

    return f_xy + alpha * f_r


def wrap_to_pi(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class InverseDynamicsControllerNode(Node):
    def __init__(self):
        super().__init__('inverse_dynamics_controller')

        # Pool-centered Figure-8 parameters (70% of experimental pool bounds)
        self.declare_parameter('center_x', -0.5713)
        self.declare_parameter('center_y', -1.2731)
        self.declare_parameter('span_x', 5.0292)
        self.declare_parameter('span_y', 4.8290)
        self.declare_parameter('scale_factor', 0.70)
        self.declare_parameter('period_s', 90.0)         # Slower, gentler 90 s Figure-8
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('soft_start_s', 6.0)      # Smooth 6 s velocity & heading ramp
        self.declare_parameter('start_at_closest', True)

        # Decoupled Longitudinal (surge, X_body) vs Lateral (sway, Y_body) & Yaw gains:
        # Both surge (kp_x, kd_x) and sway (kp_y, kd_y) use gentle gains + reference drag feedforward
        # so the water's natural hydrodynamic damping (Xu, Yv) prevents any oscillation or lunging.
        self.declare_parameter('kp_x', 0.90)
        self.declare_parameter('kd_x', 1.15)
        self.declare_parameter('ki_x', 0.00)
        self.declare_parameter('max_tau_u', 16.0)        # Cap longitudinal surge force [N] for smooth forward motion
        self.declare_parameter('kp_y', 0.85)
        self.declare_parameter('kd_y', 1.10)
        self.declare_parameter('max_tau_v', 12.0)        # Cap lateral sway force [N] to prevent side-to-side oscillation
        self.declare_parameter('kp_yaw', 2.00)
        self.declare_parameter('kd_yaw', 2.40)
        self.declare_parameter('max_tau_r', 7.0)         # Cap yaw torque [N*m] so rotation never steals XY authority

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
        self.start_at_closest = bool(self.get_parameter('start_at_closest').value)

        self.kp_x = float(self.get_parameter('kp_x').value)
        self.kd_x = float(self.get_parameter('kd_x').value)
        self.ki_x = float(self.get_parameter('ki_x').value)
        self.max_tau_u = float(self.get_parameter('max_tau_u').value)
        self.kp_y = float(self.get_parameter('kp_y').value)
        self.kd_y = float(self.get_parameter('kd_y').value)
        self.max_tau_v = float(self.get_parameter('max_tau_v').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.max_tau_r = float(self.get_parameter('max_tau_r').value)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.visual_odom_topic = self.get_parameter('visual_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.pwm_topic = self.get_parameter('pwm_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.model = DEFAULT_FOSSEN_MODEL.copy()
        self._load_motor_mapping()

        # Lightweight Publishers (NO NavPath arrays or executed trail markers)
        self.pwm_pub = self.create_publisher(Float32MultiArray, self.pwm_topic, 10)
        self.auto_pub = self.create_publisher(Bool, '/usv5/autonomous_active', 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 5)

        # Subscribers (ONLY filtered odometry + onboard IMU -> minimal WiFi & ROS overhead)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 20)

        # Real-time state variables
        self.has_odom = False
        self.has_imu_r = False
        self.x = self.center_x
        self.y = self.center_y
        self.yaw = 0.0
        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.prev_odom_time = None

        # Inertial and body-frame filtered velocities
        self.vx_odom = 0.0
        self.vy_odom = 0.0
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0

        # Position integral state (in odom XY frame) with anti-windup
        self.int_ex = 0.0
        self.int_ey = 0.0

        self.start_wall_time = None
        self.t_offset = 0.0
        self.initial_yaw = 0.0
        self.tick_count = 0
        self.cmd_filtered = np.zeros(4, dtype=float)
        self.log_rows = []

        # 30 Hz Control Timer
        self.create_timer(self.dt, self.control_loop_tick)

        self.get_logger().info(
            f"3-DOF Fossen Inverse Dynamics Controller initialized at {self.rate_hz:.0f} Hz:\n"
            f"  Pool Center (X_c, Y_c) : ({self.center_x:.4f}, {self.center_y:.4f}) m\n"
            f"  70% Figure-8 Semi-Axes : A_x = {self.ax_8:.4f} m, A_y = {self.ay_8:.4f} m\n"
            f"  Figure-8 Period        : {self.period_s:.1f} s (closest start: {self.start_at_closest})\n"
            f"  Surge Gains (X_body)   : kp_x={self.kp_x}, kd_x={self.kd_x}, max_tau_u={self.max_tau_u} N\n"
            f"  Sway Gains  (Y_body)   : kp_y={self.kp_y}, kd_y={self.kd_y}, max_tau_v={self.max_tau_v} N\n"
            f"  Yaw Gains   (Psi)      : kp_yaw={self.kp_yaw}, kd_yaw={self.kd_yaw}, max_tau_r={self.max_tau_r} N*m\n"
            f"  RViz Output            : Single moving sphere on {self.marker_topic} (15 Hz)"
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

    def find_closest_trajectory_time(self, x0: float, y0: float, yaw0: float) -> float:
        """Finds t_offset in [0, period_s) on the Figure-8 closest to the boat's current pose."""
        w = self.omega
        Ax, Ay = self.ax_8, self.ay_8
        xc, yc = self.center_x, self.center_y
        n_samples = 1000
        best_t = 0.0
        best_score = 1e18
        for i in range(n_samples):
            t_cand = (i / n_samples) * self.period_s
            xd = xc + Ax * math.sin(w * t_cand)
            yd = yc + Ay * math.sin(2.0 * w * t_cand)
            dxd = Ax * w * math.cos(w * t_cand)
            dyd = 2.0 * Ay * w * math.cos(2.0 * w * t_cand)
            psid = math.atan2(dyd, dxd)
            d2 = (xd - x0) ** 2 + (yd - y0) ** 2
            dpsi = wrap_to_pi(psid - yaw0)
            score = d2 + (0.20 ** 2) * (1.0 - math.cos(dpsi))
            if score < best_score:
                best_score = score
                best_t = t_cand
        return best_t

    def imu_cb(self, msg: Imu):
        self.r = 0.65 * self.r + 0.35 * float(msg.angular_velocity.z)
        self.has_imu_r = True

    def odom_cb(self, msg: Odometry):
        now_s = time.monotonic()
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        tw = msg.twist.twist

        x_new = float(pos.x)
        y_new = float(pos.y)
        yaw_new = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)

        u_ekf = float(tw.linear.x)
        v_ekf = float(tw.linear.y)
        r_ekf = float(tw.angular.z)

        cos_y = math.cos(yaw_new)
        sin_y = math.sin(yaw_new)

        if abs(u_ekf) > 1e-9 or abs(v_ekf) > 1e-9 or abs(r_ekf) > 1e-9:
            # Convert EKF body velocity to inertial frame and filter in inertial frame
            # so pure boat rotation does not create artificial translational velocity spikes
            vx_meas = float(np.clip(cos_y * u_ekf - sin_y * v_ekf, -1.0, 1.0))
            vy_meas = float(np.clip(sin_y * u_ekf + cos_y * v_ekf, -1.0, 1.0))
            self.vx_odom = 0.72 * self.vx_odom + 0.28 * vx_meas
            self.vy_odom = 0.72 * self.vy_odom + 0.28 * vy_meas
            if not self.has_imu_r:
                self.r = 0.70 * self.r + 0.30 * float(np.clip(r_ekf, -1.8, 1.8))
        else:
            if self.prev_odom_time is not None:
                dt_odom = max(1e-3, now_s - self.prev_odom_time)
                vx_raw = float(np.clip((x_new - self.prev_x) / dt_odom, -1.0, 1.0))
                vy_raw = float(np.clip((y_new - self.prev_y) / dt_odom, -1.0, 1.0))
                self.vx_odom = 0.75 * self.vx_odom + 0.25 * vx_raw
                self.vy_odom = 0.75 * self.vy_odom + 0.25 * vy_raw
                if not self.has_imu_r:
                    dyaw = wrap_to_pi(yaw_new - self.prev_yaw)
                    self.r = 0.70 * self.r + 0.30 * float(np.clip(dyaw / dt_odom, -1.8, 1.8))

        self.u = self.vx_odom * cos_y + self.vy_odom * sin_y
        self.v = -self.vx_odom * sin_y + self.vy_odom * cos_y

        self.x = x_new
        self.y = y_new
        self.yaw = yaw_new
        self.prev_x = x_new
        self.prev_y = y_new
        self.prev_yaw = yaw_new
        self.prev_odom_time = now_s
        self.has_odom = True

    def _delete_legacy_markers_once(self):
        for ns_del, id_del in (("figure8_start", 11), ("usv_executed_trail", 13)):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.ns = ns_del
            m.id = id_del
            m.action = Marker.DELETE
            self.marker_pub.publish(m)

    def control_loop_tick(self):
        if not self.has_odom:
            return

        now_s = time.monotonic()
        if self.start_wall_time is None:
            self.start_wall_time = now_s
            self.initial_yaw = self.yaw
            if self.start_at_closest:
                self.t_offset = self.find_closest_trajectory_time(self.x, self.y, self.yaw)
            else:
                self.t_offset = 0.0
            eta0, _, _ = self.evaluate_figure8(self.t_offset)
            self._delete_legacy_markers_once()
            self.get_logger().info(
                f"First odometry at ({self.x:.2f}, {self.y:.2f}, yaw={math.degrees(self.yaw):.1f} deg). "
                f"Starting at closest Figure-8 point ({eta0[0]:.2f}, {eta0[1]:.2f}) [t_offset={self.t_offset:.2f} s]."
            )

        t_elapsed = now_s - self.start_wall_time

        # Smooth cosine ramp over self.soft_start_s seconds for reference velocities & heading alignment
        if self.soft_start_s > 0.0 and t_elapsed < self.soft_start_s:
            s_ramp = 0.5 * (1.0 - math.cos(math.pi * t_elapsed / self.soft_start_s))
        else:
            s_ramp = 1.0

        # Slowly advance trajectory time during soft_start_s (integral of s_ramp(t) is 0.5*t at end of ramp)
        if self.soft_start_s > 0.0 and t_elapsed < self.soft_start_s:
            t_effective = 0.5 * t_elapsed - (self.soft_start_s / (2.0 * math.pi)) * math.sin(math.pi * t_elapsed / self.soft_start_s)
        else:
            t_effective = t_elapsed - 0.5 * self.soft_start_s

        eta_d, deta_d, ddeta_d = self.evaluate_figure8(self.t_offset + max(0.0, t_effective))
        deta_d = s_ramp * deta_d
        ddeta_d = (s_ramp ** 2) * ddeta_d

        # Smoothly blend desired yaw from initial_yaw to Figure-8 tangent over soft_start_s
        # so the boat holds (x, y) first and rotates gently without any initial step jump
        psi_tangent = float(eta_d[2])
        psi_d = wrap_to_pi(self.initial_yaw + s_ramp * wrap_to_pi(psi_tangent - self.initial_yaw))
        eta_d[2] = psi_d

        # 1. Inertial Position & Velocity Tracking Error projected to Body Frame (Surge X_b vs Sway Y_b)
        ex = float(eta_d[0] - self.x)
        ey = float(eta_d[1] - self.y)
        err_xy = math.hypot(ex, ey)
        if err_xy > 0.70:
            ex *= 0.70 / err_xy
            ey *= 0.70 / err_xy

        dex = float(deta_d[0] - self.vx_odom)
        dey = float(deta_d[1] - self.vy_odom)

        c_psi = math.cos(self.yaw)
        s_psi = math.sin(self.yaw)

        # Body-frame longitudinal (surge) and lateral (sway) errors
        ex_b = c_psi * ex + s_psi * ey
        ey_b = -s_psi * ex + c_psi * ey
        dex_b = c_psi * dex + s_psi * dey
        dey_b = -s_psi * dex + c_psi * dey

        # Body-frame reference velocity & acceleration
        u_d_b = c_psi * float(deta_d[0]) + s_psi * float(deta_d[1])
        v_d_b = -s_psi * float(deta_d[0]) + c_psi * float(deta_d[1])
        ax_d_b = c_psi * float(ddeta_d[0]) + s_psi * float(ddeta_d[1])
        ay_d_b = -s_psi * float(ddeta_d[0]) + c_psi * float(ddeta_d[1])

        # Anti-windup longitudinal integrator only (no lateral integrator windup -> zero side-to-side bounce)
        self.int_ex = float(np.clip(self.int_ex + ex_b * self.dt, -0.25, 0.25))

        # Decoupled body-frame desired accelerations (gentle kp_y & kd_y on lateral sway axis!)
        au_body = ax_d_b + self.kp_x * ex_b + self.kd_x * dex_b + self.ki_x * self.int_ex
        av_body = ay_d_b + self.kp_y * ey_b + self.kd_y * dey_b

        # 2. Yaw Tracking Error (soft-clamped so yaw never overpowers XY position)
        epsi = wrap_to_pi(psi_d - self.yaw)
        epsi_clamped = float(np.clip(epsi, -0.50, 0.50))
        depsi = float(deta_d[2] - self.r)
        ar_body = float(ddeta_d[2]) + self.kp_yaw * epsi_clamped + self.kd_yaw * depsi

        # 3. 3-DOF Fossen Feedforward + Feedback Generalized Forces
        # Note: Both longitudinal (u_d_b) and lateral (v_d_b) drag feedforwards use the reference
        # velocities so the water's natural hydrodynamic drag (-Xu*u, -Yv*v) passively damps overshoots!
        m = self.model
        tau_u = m["m11"] * au_body + (m["Xu"] + m["Xuu"] * abs(u_d_b)) * u_d_b
        tau_u = float(np.clip(tau_u, -self.max_tau_u, self.max_tau_u))
        tau_v = m["m22"] * av_body + (m["Yv"] + m["Yvv"] * abs(v_d_b)) * v_d_b
        tau_v = float(np.clip(tau_v, -self.max_tau_v, self.max_tau_v))
        tau_r = m["m33"] * ar_body + (m["Nr"] + m["Nrr"] * abs(deta_d[2])) * float(deta_d[2])
        tau_r = float(np.clip(tau_r, -self.max_tau_r, self.max_tau_r))

        # 4. Position-Prioritized & Symmetric Thruster Allocation -> Richards Inverse
        f_motors = allocate_priority_xy(tau_u, tau_v, tau_r)
        cmd_raw = thrust_to_cmd_richards(f_motors)
        self.cmd_filtered = 0.65 * self.cmd_filtered + 0.35 * cmd_raw
        u_FR, u_FL, u_BR, u_BL = (
            float(self.cmd_filtered[0]),
            float(self.cmd_filtered[1]),
            float(self.cmd_filtered[2]),
            float(self.cmd_filtered[3]),
        )

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

        # 5. Ultra-lightweight RViz target sphere at 15 Hz (every 2nd tick, < 100 bytes)
        self.tick_count += 1
        if self.tick_count % 2 == 0:
            self._publish_target_sphere(float(eta_d[0]), float(eta_d[1]))

        # Log sample in memory (written to CSV only on shutdown)
        c_d = math.cos(psi_d)
        s_d = math.sin(psi_d)
        u_ref = c_d * deta_d[0] + s_d * deta_d[1]
        v_ref = -s_d * deta_d[0] + c_d * deta_d[1]
        r_ref = deta_d[2]
        self.log_rows.append([
            f"{t_elapsed:.4f}",
            f"{eta_d[0]:.5f}", f"{eta_d[1]:.5f}", f"{psi_d:.5f}",
            f"{self.x:.5f}", f"{self.y:.5f}", f"{self.yaw:.5f}",
            f"{u_ref:.5f}", f"{v_ref:.5f}", f"{r_ref:.5f}",
            f"{self.u:.5f}", f"{self.v:.5f}", f"{self.r:.5f}",
            f"{tau_u:.4f}", f"{tau_v:.4f}", f"{tau_r:.4f}",
            f"{out_data[0]:.4f}", f"{out_data[1]:.4f}", f"{out_data[2]:.4f}", f"{out_data[3]:.4f}"
        ])

    def _publish_target_sphere(self, xd: float, yd: float):
        target_marker = Marker()
        # Use Time(0, 0) so RViz renders immediately on the latest 'odom' frame without clock-skew queue lag
        target_marker.header.frame_id = self.frame_id
        target_marker.ns = "figure8_target"
        target_marker.id = 12
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = xd
        target_marker.pose.position.y = yd
        target_marker.pose.position.z = -0.04
        target_marker.pose.orientation.w = 1.0
        target_marker.scale.x = 0.09
        target_marker.scale.y = 0.09
        target_marker.scale.z = 0.09
        target_marker.color = ColorRGBA(r=1.0, g=0.82, b=0.0, a=0.95)  # Bright Gold Sphere
        self.marker_pub.publish(target_marker)

    def stop_thrusters_and_save_log(self):
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
