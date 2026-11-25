#!/usr/bin/env python3
"""
smc_controller.py
=================
Real-Time 30 Hz 3-DOF Full-Actuated Sliding Mode Controller (SMC) with optional
Control Barrier Function (CBF) Obstacle Avoidance and Virtual-Time Reference Governor
for the PlaDyPos USV inside the Centered 70% Pool Workspace.

Supported Trajectories (`trajectory_type`):
  - 'figure8' (or 'figure_eight'): Centered 70% Figure-8 (Lemniscate)
  - 'spiral'                     : Centered 70% Archimedean Spiral
  - 'clover'                     : Centered 70% Three-Leaf Clover (3-Petal Rose)

Supported Modes (`enable_obstacles`):
  - False: Nominal 3-DOF Full-Actuated SMC trajectory tracking
  - True : 3-DOF Full-Actuated SMC + HOCBF Obstacle Avoidance + Reference Governor
           (also publishes the 2 buoys and their CBF safety zones to RViz on /pool_boundary)
"""

import csv
import json
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
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Bool, ColorRGBA
from visualization_msgs.msg import Marker


# ==============================================================================
# 1. DEFAULT 9-PARAMETER WHITE-BOX FOSSEN MODEL & X-THRUSTER CONFIGURATION
#    (Loaded dynamically from model_params.json and thrusters_config.json)
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

D_ARM = 0.65 / math.sqrt(2.0)  # 0.459619 m
COS45 = math.cos(math.radians(45.0))
SIN45 = math.sin(math.radians(45.0))

DEFAULT_PHI_ALLOC = np.array([
    [ COS45,  COS45, -COS45, -COS45],
    [ SIN45, -SIN45,  SIN45, -SIN45],
    [ D_ARM, -D_ARM, -D_ARM,  D_ARM]
], dtype=float)

DEFAULT_RICHARDS_PARAMS = {
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

# Default obstacle configurations inside the 70% pool workspace for each trajectory
OBSTACLES_BY_TRAJECTORY = {
    "figure8": (
        {"x": 0.784,  "y": -2.933, "radius": 0.14, "name": "buoy_1"},
        {"x": -1.927, "y": -2.933, "radius": 0.12, "name": "buoy_2"},
    ),
    "spiral": (
        {"x": -1.114, "y": -1.794, "radius": 0.12, "name": "buoy_1"},
        {"x": -0.042, "y": -0.276, "radius": 0.14, "name": "buoy_2"},
    ),
    "clover": (
        {"x": -0.871, "y": -0.152, "radius": 0.14, "name": "buoy_1"},
        {"x": -1.451, "y": -2.118, "radius": 0.12, "name": "buoy_2"},
    ),
}


def load_json_configs():
    """Loads model_params.json and thrusters_config.json from package share or workspace."""
    model = DEFAULT_FOSSEN_MODEL.copy()
    richards = DEFAULT_RICHARDS_PARAMS.copy()
    phi_alloc = DEFAULT_PHI_ALLOC.copy()

    candidate_dirs = []
    try:
        pkg_share = Path(get_package_share_directory('pladypos')) / 'config'
        candidate_dirs.append(pkg_share)
    except Exception:
        pass
    candidate_dirs.extend([
        Path(__file__).resolve().parent.parent.parent / 'config',
        Path('/home/brayan/ros2_ws/src/smc_full_actuated'),
        Path('/home/brayan/ros2_ws/src/pladypos/config'),
        Path('/config'),
    ])

    for d in candidate_dirs:
        m_path = d / 'model_params.json'
        if m_path.exists():
            try:
                with open(m_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                params = data.get('parameters', data)
                for k in DEFAULT_FOSSEN_MODEL:
                    if k in params:
                        model[k] = float(params[k])
                break
            except Exception:
                pass

    for d in candidate_dirs:
        t_path = d / 'thrusters_config.json'
        if t_path.exists():
            try:
                with open(t_path, 'r', encoding='utf-8') as f:
                    tdata = json.load(f)
                if 'thruster_curve' in tdata:
                    for k, v in tdata['thruster_curve'].items():
                        if isinstance(v, (int, float)):
                            richards[k] = float(v)
                if 'thrusters' in tdata:
                    thrusters = sorted(tdata['thrusters'], key=lambda item: item['index'])
                    cols = []
                    for m in thrusters:
                        cx = float(m['force_direction_body']['cos_alpha'])
                        sy = float(m['force_direction_body']['sin_alpha'])
                        arm = float(m['moment_arm_m'])
                        cols.append([cx, sy, arm])
                    phi_alloc = np.column_stack(cols).astype(float)
                break
            except Exception:
                pass

    phi_pinv = np.linalg.pinv(phi_alloc)
    return model, richards, phi_alloc, phi_pinv


def thrust_to_cmd_richards(T_des: np.ndarray, richards_params: dict) -> np.ndarray:
    """Analytical inverse of the asymmetric Richards thrust model: Newtons -> cmd in [-1, 1]."""
    p = richards_params
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


def allocate_priority_xy(tau_u: float, tau_v: float, tau_r: float, phi_pinv: np.ndarray) -> np.ndarray:
    """
    Allocates generalized forces [tau_u, tau_v, tau_r] across the 4 X-thrusters
    with strict priority for (x, y) translation over yaw rotation, bounded symmetrically
    by F_SYM_MAX = 28.0 N so yaw rotation NEVER induces parasitic (x, y) drift.
    """
    f_xy = phi_pinv @ np.array([tau_u, tau_v, 0.0], dtype=float)
    max_xy = float(np.max(np.abs(f_xy)))
    xy_limit = 24.0
    if max_xy > xy_limit:
        f_xy = f_xy * (xy_limit / max_xy)

    f_r = phi_pinv @ np.array([0.0, 0.0, tau_r], dtype=float)
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


def sat(val):
    return np.clip(val, -1.0, 1.0)


def project_cbf_3dof(z_nominal: np.ndarray, constraints: list, tol: float = 1e-9):
    """Minimum-norm Euclidean projection of z = [Tu, Tv, Tr] onto a_i @ z >= b_i."""
    active = [c for c in constraints if np.linalg.norm(c["a"]) > tol]
    if not active:
        return z_nominal.copy(), False

    A = np.vstack([c["a"] for c in active])
    b = np.array([c["b"] for c in active], dtype=float)

    def is_feasible(z):
        return bool(np.all(A @ z >= b - 1e-7))

    if is_feasible(z_nominal):
        return z_nominal.copy(), False

    candidates = []
    for row, rhs in zip(A, b):
        denom = float(np.dot(row, row))
        if denom > tol:
            candidates.append(z_nominal + ((rhs - float(row @ z_nominal)) / denom) * row)

    for i in range(len(A)):
        for j in range(i + 1, len(A)):
            A_pair = np.vstack([A[i], A[j]])
            b_pair = np.array([b[i], b[j]], dtype=float)
            gram = A_pair @ A_pair.T
            if abs(float(np.linalg.det(gram))) > tol:
                lam = np.linalg.solve(gram, b_pair - A_pair @ z_nominal)
                candidates.append(z_nominal + A_pair.T @ lam)

    feasible = [c for c in candidates if is_feasible(c)]
    if not feasible:
        return z_nominal.copy(), False

    z_cbf = min(feasible, key=lambda c: float(np.linalg.norm(c - z_nominal)))
    return z_cbf, True


class SMCControllerNode(Node):
    def __init__(self):
        super().__init__('smc_controller')

        # 1. Pool & Trajectory Parameters
        self.declare_parameter('center_x', -0.5713)
        self.declare_parameter('center_y', -1.2731)
        self.declare_parameter('span_x', 5.0292)
        self.declare_parameter('span_y', 4.8290)
        self.declare_parameter('scale_factor', 0.70)
        self.declare_parameter('trajectory_type', 'figure8')   # 'figure8', 'spiral', or 'clover'
        self.declare_parameter('period_s', 90.0)               # Trajectory period [s]
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('soft_start_s', 6.0)
        self.declare_parameter('start_at_closest', True)

        # 2. 3-DOF Full-Actuated SMC Parameters (3 sliding surfaces: s_u, s_v, s_r)
        self.declare_parameter('k_pos', 0.65)
        self.declare_parameter('k_pos_v', 0.45)
        self.declare_parameter('k_psi', 0.90)
        self.declare_parameter('lambda_u', 0.70)
        self.declare_parameter('lambda_v', 0.50)
        self.declare_parameter('lambda_r', 1.20)
        self.declare_parameter('k_s_u', 0.09)
        self.declare_parameter('k_s_v', 0.05)
        self.declare_parameter('k_s_r', 0.15)
        self.declare_parameter('phi_u', 0.25)
        self.declare_parameter('phi_v', 0.30)
        self.declare_parameter('phi_r', 0.25)
        self.declare_parameter('max_tau_u', 14.0)
        self.declare_parameter('max_tau_v', 8.5)
        self.declare_parameter('max_tau_r', 6.0)

        # 3. Obstacles (CBF + Virtual-Time Reference Governor) Parameters
        self.declare_parameter('enable_obstacles', False)
        self.declare_parameter('cbf_margin', 0.21)
        self.declare_parameter('cbf_alpha_1', 2.0)
        self.declare_parameter('cbf_alpha_2', 2.0)
        self.declare_parameter('governor_gamma_min', 0.50)
        self.declare_parameter('governor_h_slow', 1.20)
        self.declare_parameter('governor_h_stop', 0.08)

        # 4. ROS Topics
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

        raw_traj = str(self.get_parameter('trajectory_type').value).strip().lower()
        if raw_traj in ('figure8', 'figure_eight', '8'):
            self.trajectory_type = 'figure8'
        elif raw_traj in ('spiral', 'espiral'):
            self.trajectory_type = 'spiral'
        elif raw_traj in ('clover', 'trebol', 'trefoil'):
            self.trajectory_type = 'clover'
        else:
            self.trajectory_type = 'figure8'

        self.ax_70 = 0.5 * self.span_x * self.scale_factor
        self.ay_70 = 0.5 * self.span_y * self.scale_factor
        self.period_s = float(self.get_parameter('period_s').value)
        self.omega = 2.0 * math.pi / self.period_s
        self.rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.dt = 1.0 / self.rate_hz
        self.soft_start_s = float(self.get_parameter('soft_start_s').value)
        self.start_at_closest = bool(self.get_parameter('start_at_closest').value)
        if self.trajectory_type == 'spiral':
            # Spiral always starts from its inner ring at t = 0
            self.start_at_closest = False

        self.k_pos = float(self.get_parameter('k_pos').value)
        self.k_pos_v = float(self.get_parameter('k_pos_v').value)
        self.k_psi = float(self.get_parameter('k_psi').value)
        self.lambda_u = float(self.get_parameter('lambda_u').value)
        self.lambda_v = float(self.get_parameter('lambda_v').value)
        self.lambda_r = float(self.get_parameter('lambda_r').value)
        self.k_s_u = float(self.get_parameter('k_s_u').value)
        self.k_s_v = float(self.get_parameter('k_s_v').value)
        self.k_s_r = float(self.get_parameter('k_s_r').value)
        self.phi_u = float(self.get_parameter('phi_u').value)
        self.phi_v = float(self.get_parameter('phi_v').value)
        self.phi_r = float(self.get_parameter('phi_r').value)
        self.max_tau_u = float(self.get_parameter('max_tau_u').value)
        self.max_tau_v = float(self.get_parameter('max_tau_v').value)
        self.max_tau_r = float(self.get_parameter('max_tau_r').value)

        self.enable_obstacles = bool(self.get_parameter('enable_obstacles').value)
        self.cbf_margin = float(self.get_parameter('cbf_margin').value)
        self.cbf_alpha_1 = float(self.get_parameter('cbf_alpha_1').value)
        self.cbf_alpha_2 = float(self.get_parameter('cbf_alpha_2').value)
        self.gamma_min = float(self.get_parameter('governor_gamma_min').value)
        self.h_slow = float(self.get_parameter('governor_h_slow').value)
        self.h_stop = float(self.get_parameter('governor_h_stop').value)

        self.obstacles = (
            OBSTACLES_BY_TRAJECTORY.get(self.trajectory_type, OBSTACLES_BY_TRAJECTORY["figure8"])
            if self.enable_obstacles
            else ()
        )

        self.odom_topic = self.get_parameter('odom_topic').value
        self.visual_odom_topic = self.get_parameter('visual_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.pwm_topic = self.get_parameter('pwm_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.model, self.richards_params, self.phi_alloc, self.phi_pinv = load_json_configs()
        self._load_motor_mapping()

        # Publishers
        self.pwm_pub = self.create_publisher(Float32MultiArray, self.pwm_topic, 10)
        self.auto_pub = self.create_publisher(Bool, '/usv5/autonomous_active', 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)

        # Subscribers
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

        self.vx_odom = 0.0
        self.vy_odom = 0.0
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0

        self.start_wall_time = None
        self.t_offset = 0.0
        self.sigma = 0.0
        self.gamma = 1.0
        self.prev_tick_time = None
        self.initial_yaw = 0.0
        self.tick_count = 0
        self.cmd_filtered = np.zeros(4, dtype=float)
        self.log_rows = []

        self.create_timer(self.dt, self.control_loop_tick)

        self.get_logger().info(
            f"3-DOF Full-Actuated SMC Controller initialized at {self.rate_hz:.0f} Hz:\n"
            f"  Trajectory Type        : {self.trajectory_type.upper()} (Period = {self.period_s:.1f} s, closest start: {self.start_at_closest})\n"
            f"  Obstacles (CBF + Gov)  : {'ENABLED (' + str(len(self.obstacles)) + ' buoys)' if self.enable_obstacles else 'DISABLED (Ideal Tracking)'}\n"
            f"  Pool Center (X_c, Y_c) : ({self.center_x:.4f}, {self.center_y:.4f}) m | Semi-Axes: ({self.ax_70:.3f}, {self.ay_70:.3f}) m\n"
            f"  SMC Gains (s_u,s_v,s_r): lambda=({self.lambda_u}, {self.lambda_v}, {self.lambda_r}) | k_s=({self.k_s_u}, {self.k_s_v}, {self.k_s_r})\n"
            f"  Force Limits           : max_tau_u={self.max_tau_u} N, max_tau_v={self.max_tau_v} N, max_tau_r={self.max_tau_r} N*m"
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

    def evaluate_trajectory(self, t_sec: float, time_scale: float = 1.0):
        """
        Evaluates [xd, yd, psid], [dxd, dyd, dpsid], [ddxd, ddyd, ddpsid]
        at virtual time t_sec with rate time_scale = gamma for ('figure8', 'spiral', 'clover').
        """
        Ax, Ay = self.ax_70, self.ay_70
        xc, yc = self.center_x, self.center_y
        T_p = self.period_s
        w = self.omega

        if self.trajectory_type == 'spiral':
            # Archimedean spiral cycling smoothly between inner (0.18) and outer (1.00) radius
            # over period T_p (2 turns per period)
            t_mod = t_sec % T_p
            s0, s1 = 0.18, 1.00
            a_s = (s1 - s0) / T_p
            omega_s = 2.0 * math.pi * 2.0 / T_p

            scale = s0 + a_s * t_mod
            th = omega_s * t_mod
            c, s = math.cos(th), math.sin(th)

            xd = xc + Ax * scale * c
            yd = yc + Ay * scale * s
            dxd = Ax * (a_s * c - scale * omega_s * s)
            dyd = Ay * (a_s * s + scale * omega_s * c)
            ddxd = Ax * (-2.0 * a_s * omega_s * s - scale * (omega_s ** 2) * c)
            ddyd = Ay * (2.0 * a_s * omega_s * c - scale * (omega_s ** 2) * s)
            dddxd = Ax * (-3.0 * a_s * (omega_s ** 2) * c + scale * (omega_s ** 3) * s)
            dddyd = Ay * (-3.0 * a_s * (omega_s ** 2) * s - scale * (omega_s ** 3) * c)

        elif self.trajectory_type == 'clover':
            # Three-Leaf Clover (R(th) = A * cos(3*th)) inside 70% pool workspace:
            #   xd = xc + 0.5 * Ax * (cos(2*w*t) + cos(4*w*t))
            #   yd = yc + 0.5 * Ay * (sin(4*w*t) - sin(2*w*t))
            p2 = 2.0 * w * t_sec
            p4 = 4.0 * w * t_sec
            c2, s2 = math.cos(p2), math.sin(p2)
            c4, s4 = math.cos(p4), math.sin(p4)
            hx, hy = 0.5 * Ax, 0.5 * Ay

            xd = xc + hx * (c2 + c4)
            yd = yc + hy * (s4 - s2)
            dxd = hx * w * (-2.0 * s2 - 4.0 * s4)
            dyd = hy * w * (4.0 * c4 - 2.0 * c2)
            ddxd = hx * (w ** 2) * (-4.0 * c2 - 16.0 * c4)
            ddyd = hy * (w ** 2) * (-16.0 * s4 + 4.0 * s2)
            dddxd = hx * (w ** 3) * (8.0 * s2 + 64.0 * s4)
            dddyd = hy * (w ** 3) * (-64.0 * c4 + 8.0 * c2)

        else:
            # Default: Figure-8 (Lemniscate) inside 70% pool workspace
            xd = xc + Ax * math.sin(w * t_sec)
            yd = yc + Ay * math.sin(2.0 * w * t_sec)
            dxd = Ax * w * math.cos(w * t_sec)
            dyd = 2.0 * Ay * w * math.cos(2.0 * w * t_sec)
            ddxd = -Ax * (w ** 2) * math.sin(w * t_sec)
            ddyd = -4.0 * Ay * (w ** 2) * math.sin(2.0 * w * t_sec)
            dddxd = -Ax * (w ** 3) * math.cos(w * t_sec)
            dddyd = -8.0 * Ay * (w ** 3) * math.cos(2.0 * w * t_sec)

        dxd *= time_scale
        dyd *= time_scale
        ddxd *= (time_scale ** 2)
        ddyd *= (time_scale ** 2)
        dddxd *= (time_scale ** 3)
        dddyd *= (time_scale ** 3)

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
        """Finds t_offset in [0, period_s) on the active trajectory closest to the boat's current pose."""
        n_samples = 1000
        best_t = 0.0
        best_score = 1e18
        for i in range(n_samples):
            t_cand = (i / n_samples) * self.period_s
            eta_c, _, _ = self.evaluate_trajectory(t_cand)
            xd, yd, psid = float(eta_c[0]), float(eta_c[1]), float(eta_c[2])
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

    def _publish_initial_rviz_markers(self):
        for ns_del, id_del in (("figure8_start", 11), ("usv_executed_trail", 13)):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.ns = ns_del
            m.id = id_del
            m.action = Marker.DELETE
            self.marker_pub.publish(m)

        # 1. Publish the active trajectory curve (181 points at z = -0.05) so RViz displays the exact shape
        ref_marker = Marker()
        ref_marker.header.frame_id = self.frame_id
        ref_marker.ns = "figure8_reference"
        ref_marker.id = 10
        ref_marker.type = Marker.LINE_STRIP
        ref_marker.action = Marker.ADD
        ref_marker.scale.x = 0.045
        ref_marker.color = ColorRGBA(r=0.95, g=0.22, b=0.22, a=0.95)
        ref_marker.pose.orientation.w = 1.0
        n_pts = 180
        for i in range(n_pts + 1):
            t_i = (i / n_pts) * self.period_s
            eta_i, _, _ = self.evaluate_trajectory(t_i)
            pt = Point()
            pt.x = float(eta_i[0])
            pt.y = float(eta_i[1])
            pt.z = -0.05
            ref_marker.points.append(pt)
        self.marker_pub.publish(ref_marker)

        # 2. Publish or clear obstacle markers (buoys + CBF safety rings) in RViz
        if self.enable_obstacles and self.obstacles:
            for idx, obs in enumerate(self.obstacles):
                # Magenta cylinder for physical buoy
                buoy_m = Marker()
                buoy_m.header.frame_id = self.frame_id
                buoy_m.ns = "cbf_obstacles"
                buoy_m.id = 20 + idx
                buoy_m.type = Marker.CYLINDER
                buoy_m.action = Marker.ADD
                buoy_m.pose.position.x = float(obs["x"])
                buoy_m.pose.position.y = float(obs["y"])
                buoy_m.pose.position.z = 0.0
                buoy_m.pose.orientation.w = 1.0
                d_buoy = 2.0 * float(obs["radius"])
                buoy_m.scale.x = d_buoy
                buoy_m.scale.y = d_buoy
                buoy_m.scale.z = 0.16
                buoy_m.color = ColorRGBA(r=0.95, g=0.10, b=0.85, a=0.85)
                self.marker_pub.publish(buoy_m)

                # Orange circle line strip for CBF safety zone (radius + cbf_margin)
                safe_r = float(obs["radius"]) + self.cbf_margin
                ring_m = Marker()
                ring_m.header.frame_id = self.frame_id
                ring_m.ns = "cbf_safety_zones"
                ring_m.id = 30 + idx
                ring_m.type = Marker.LINE_STRIP
                ring_m.action = Marker.ADD
                ring_m.scale.x = 0.035
                ring_m.color = ColorRGBA(r=1.0, g=0.58, b=0.0, a=0.90)
                ring_m.pose.orientation.w = 1.0
                n_ring = 48
                for k in range(n_ring + 1):
                    ang = 2.0 * math.pi * (k / n_ring)
                    pt = Point()
                    pt.x = float(obs["x"]) + safe_r * math.cos(ang)
                    pt.y = float(obs["y"]) + safe_r * math.sin(ang)
                    pt.z = -0.03
                    ring_m.points.append(pt)
                self.marker_pub.publish(ring_m)
        else:
            for idx in range(4):
                for ns_obs, base_id in (("cbf_obstacles", 20), ("cbf_safety_zones", 30)):
                    m_del = Marker()
                    m_del.header.frame_id = self.frame_id
                    m_del.ns = ns_obs
                    m_del.id = base_id + idx
                    m_del.action = Marker.DELETE
                    self.marker_pub.publish(m_del)

    def _update_reference_governor(self, dt_step: float):
        """Updates virtual time rate gamma and virtual time sigma based on obstacle proximity."""
        if not self.enable_obstacles or not self.obstacles:
            self.gamma = 1.0
            self.sigma += dt_step
            return

        h_vals = []
        for obs in self.obstacles:
            safe_r = float(obs["radius"]) + self.cbf_margin
            dx = self.x - float(obs["x"])
            dy = self.y - float(obs["y"])
            h_vals.append(dx * dx + dy * dy - safe_r * safe_r)

        h_min = min(h_vals)
        if self.h_slow > self.h_stop:
            blend = float(np.clip((h_min - self.h_stop) / (self.h_slow - self.h_stop), 0.0, 1.0))
            gamma_target = self.gamma_min + (1.0 - self.gamma_min) * blend
        else:
            gamma_target = 1.0

        tau_gamma = 0.8
        self.gamma += (dt_step / max(tau_gamma, 1e-3)) * (gamma_target - self.gamma)
        self.gamma = float(np.clip(self.gamma, self.gamma_min, 1.0))
        self.sigma += self.gamma * dt_step

    def _apply_cbf_filter(self, tau_smc: np.ndarray) -> tuple:
        """Applies 3-DOF HOCBF projection around configured obstacles."""
        if not self.enable_obstacles or not self.obstacles:
            return tau_smc, False

        m = self.model
        m11, m22, m33 = m["m11"], m["m22"], m["m33"]
        Xu, Xuu = m["Xu"], m["Xuu"]
        Yv, Yvv = m["Yv"], m["Yvv"]
        c, s = math.cos(self.yaw), math.sin(self.yaw)

        u_dot_drift = (m22 * self.v * self.r - (Xu + Xuu * abs(self.u)) * self.u) / m11
        v_dot_drift = (-m11 * self.u * self.r - (Yv + Yvv * abs(self.v)) * self.v) / m22
        u_axis_drift = u_dot_drift - self.v * self.r
        v_axis_drift = v_dot_drift + self.u * self.r

        e_psi = np.array([c, s], dtype=float)
        e_perp = np.array([-s, c], dtype=float)
        rot = np.array([[c, -s], [s, c]], dtype=float)

        accel_drift = rot @ np.array([u_axis_drift, v_axis_drift], dtype=float)
        accel_map = np.column_stack([
            (1.0 / m11) * e_psi,
            (1.0 / m22) * e_perp,
            np.zeros(2, dtype=float),
        ])

        p_vec = np.array([self.x, self.y], dtype=float)
        p_dot = np.array([self.vx_odom, self.vy_odom], dtype=float)

        constraints = []
        for obs in self.obstacles:
            safe_r = float(obs["radius"]) + self.cbf_margin
            center = np.array([float(obs["x"]), float(obs["y"])], dtype=float)
            rel_p = p_vec - center
            q_mat = np.eye(2, dtype=float) / (safe_r * safe_r)
            shaped_rel = q_mat @ rel_p

            h = float(rel_p @ shaped_rel - 1.0)
            h_dot = float(2.0 * shaped_rel @ p_dot)
            h_ddot_drift = float(2.0 * (p_dot @ q_mat @ p_dot + shaped_rel @ accel_drift))
            h_ddot_map = 2.0 * shaped_rel @ accel_map

            a_vec = h_ddot_map
            b_val = -(
                h_ddot_drift
                + (self.cbf_alpha_1 + self.cbf_alpha_2) * h_dot
                + self.cbf_alpha_1 * self.cbf_alpha_2 * h
            )
            constraints.append({"a": a_vec, "b": b_val, "h": h})

        return project_cbf_3dof(tau_smc, constraints)

    def control_loop_tick(self):
        if not self.has_odom:
            return

        now_s = time.monotonic()
        if self.start_wall_time is None:
            self.start_wall_time = now_s
            self.prev_tick_time = now_s
            self.initial_yaw = self.yaw
            if self.start_at_closest:
                self.t_offset = self.find_closest_trajectory_time(self.x, self.y, self.yaw)
            else:
                self.t_offset = 0.0
            self.sigma = 0.0
            self.gamma = 1.0
            eta0, _, _ = self.evaluate_trajectory(self.t_offset)
            self._publish_initial_rviz_markers()
            self.get_logger().info(
                f"First odometry at ({self.x:.2f}, {self.y:.2f}, yaw={math.degrees(self.yaw):.1f} deg). "
                f"Starting {self.trajectory_type.upper()} at ({eta0[0]:.2f}, {eta0[1]:.2f}) [t_offset={self.t_offset:.2f} s]."
            )

        dt_tick = max(1e-3, min(0.1, now_s - self.prev_tick_time))
        self.prev_tick_time = now_s
        t_elapsed = now_s - self.start_wall_time

        # Smooth cosine ramp over self.soft_start_s seconds
        if self.soft_start_s > 0.0 and t_elapsed < self.soft_start_s:
            s_ramp = 0.5 * (1.0 - math.cos(math.pi * t_elapsed / self.soft_start_s))
        else:
            s_ramp = 1.0

        self._update_reference_governor(dt_tick * s_ramp)

        eta_d, deta_d, ddeta_d = self.evaluate_trajectory(
            self.t_offset + max(0.0, self.sigma),
            time_scale=s_ramp * self.gamma,
        )

        psi_tangent = float(eta_d[2])
        psi_d = wrap_to_pi(self.initial_yaw + s_ramp * wrap_to_pi(psi_tangent - self.initial_yaw))
        eta_d[2] = psi_d

        # 1. Inertial Position Errors projected to Body Frame (Surge X_b vs Sway Y_b)
        ex = float(eta_d[0] - self.x)
        ey = float(eta_d[1] - self.y)
        err_xy = math.hypot(ex, ey)
        if err_xy > 0.70:
            ex *= 0.70 / err_xy
            ey *= 0.70 / err_xy

        c_psi = math.cos(self.yaw)
        s_psi = math.sin(self.yaw)

        # Body-frame longitudinal (surge) and lateral (sway) position errors
        ex_b = c_psi * ex + s_psi * ey
        ey_b = -s_psi * ex + c_psi * ey

        # Body-frame reference velocities and accelerations
        u_d_b = c_psi * float(deta_d[0]) + s_psi * float(deta_d[1])
        v_d_b = -s_psi * float(deta_d[0]) + c_psi * float(deta_d[1])
        ax_d_b = c_psi * float(ddeta_d[0]) + s_psi * float(ddeta_d[1])
        ay_d_b = -s_psi * float(ddeta_d[0]) + c_psi * float(ddeta_d[1])

        # Decoupled desired body velocities (gentler k_pos_v on lateral sway axis prevents side-to-side rocking)
        u_cmd = u_d_b + self.k_pos * ex_b
        v_cmd = v_d_b + self.k_pos_v * ey_b

        epsi = wrap_to_pi(psi_d - self.yaw)
        epsi_clamped = float(np.clip(epsi, -0.50, 0.50))
        r_cmd = float(deta_d[2]) + self.k_psi * epsi_clamped

        # 2. 3-DOF Sliding Surfaces (Surge s_u, Sway s_v, Yaw s_r)
        s_u = self.u - u_cmd
        s_v = self.v - v_cmd
        s_r = self.r - r_cmd

        # 3. 3-DOF SMC Reaching Law in Body Accelerations (without noisy velocity derivative amplification)
        u_dot_des = ax_d_b - self.lambda_u * s_u - self.k_s_u * float(sat(s_u / self.phi_u))
        v_dot_des = ay_d_b - self.lambda_v * s_v - self.k_s_v * float(sat(s_v / self.phi_v))
        r_dot_des = float(ddeta_d[2]) - self.lambda_r * s_r - self.k_s_r * float(sat(s_r / self.phi_r))

        # 4. 9-Parameter Fossen Dynamic Inversion (using reference drag feedforward so water passively damps sway)
        m = self.model
        tau_u = m["m11"] * u_dot_des + (m["Xu"] + m["Xuu"] * abs(u_d_b)) * u_d_b
        tau_v = m["m22"] * v_dot_des + (m["Yv"] + m["Yvv"] * abs(v_d_b)) * v_d_b
        tau_r = m["m33"] * r_dot_des + (m["Nr"] + m["Nrr"] * abs(float(deta_d[2]))) * float(deta_d[2])

        # 5. Optional 3-DOF CBF Obstacle Avoidance Filter
        tau_nominal = np.array([tau_u, tau_v, tau_r], dtype=float)
        tau_safe, cbf_active = self._apply_cbf_filter(tau_nominal)

        tau_u = float(np.clip(tau_safe[0], -self.max_tau_u, self.max_tau_u))
        tau_v = float(np.clip(tau_safe[1], -self.max_tau_v, self.max_tau_v))
        tau_r = float(np.clip(tau_safe[2], -self.max_tau_r, self.max_tau_r))

        # 6. Position-Prioritized & Symmetric Thruster Allocation -> Richards Inverse
        f_motors = allocate_priority_xy(tau_u, tau_v, tau_r, self.phi_pinv)
        cmd_raw = thrust_to_cmd_richards(f_motors, self.richards_params)
        self.cmd_filtered = 0.72 * self.cmd_filtered + 0.28 * cmd_raw
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

        # 8. Ultra-lightweight RViz target sphere at 15 Hz (every 2nd tick)
        self.tick_count += 1
        if self.tick_count % 2 == 0:
            self._publish_target_sphere(float(eta_d[0]), float(eta_d[1]), float(psi_d))

        c_d = math.cos(psi_d)
        s_d = math.sin(psi_d)
        u_ref = c_d * float(deta_d[0]) + s_d * float(deta_d[1])
        v_ref = -s_d * float(deta_d[0]) + c_d * float(deta_d[1])
        r_ref = float(deta_d[2])
        self.log_rows.append([
            f"{t_elapsed:.4f}",
            f"{eta_d[0]:.5f}", f"{eta_d[1]:.5f}", f"{psi_d:.5f}",
            f"{self.x:.5f}", f"{self.y:.5f}", f"{self.yaw:.5f}",
            f"{u_ref:.5f}", f"{v_ref:.5f}", f"{r_ref:.5f}",
            f"{self.u:.5f}", f"{self.v:.5f}", f"{self.r:.5f}",
            f"{tau_u:.4f}", f"{tau_v:.4f}", f"{tau_r:.4f}",
            f"{out_data[0]:.4f}", f"{out_data[1]:.4f}", f"{out_data[2]:.4f}", f"{out_data[3]:.4f}",
            f"{int(cbf_active)}", f"{self.gamma:.4f}"
        ])

    def _publish_target_sphere(self, xd: float, yd: float, psid: float = 0.0):
        target_marker = Marker()
        target_marker.header.frame_id = self.frame_id
        target_marker.ns = "figure8_target"
        target_marker.id = 12
        target_marker.type = Marker.SPHERE
        target_marker.action = Marker.ADD
        target_marker.pose.position.x = xd
        target_marker.pose.position.y = yd
        target_marker.pose.position.z = -0.04
        target_marker.pose.orientation.w = math.cos(0.5 * psid)
        target_marker.pose.orientation.x = 0.0
        target_marker.pose.orientation.y = 0.0
        target_marker.pose.orientation.z = math.sin(0.5 * psid)
        target_marker.scale.x = 0.09
        target_marker.scale.y = 0.09
        target_marker.scale.z = 0.09
        target_marker.color = ColorRGBA(r=1.0, g=0.82, b=0.0, a=0.95)
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
            obs_tag = "obstacles" if self.enable_obstacles else "ideal"
            csv_file = out_dir / f"realtime_smc_{self.trajectory_type}_{obs_tag}_{ts}.csv"
            latest_file = out_dir / "realtime_smc_30Hz.csv"
            header = [
                "t",
                "x_ref", "y_ref", "psi_ref",
                "x_real", "y_real", "psi_real",
                "u_ref", "v_ref", "r_ref",
                "u_real", "v_real", "r_real",
                "Tu", "Tv", "Tr",
                "cmd_m1_FR", "cmd_m2_FL", "cmd_m3_BR", "cmd_m4_BL",
                "cbf_active", "gamma"
            ]
            for target_path in (csv_file, latest_file):
                with open(target_path, "w", newline="", encoding="utf-8") as fp:
                    writer = csv.writer(fp)
                    writer.writerow(header)
                    writer.writerows(self.log_rows)
            self.get_logger().info(f"Saved {len(self.log_rows)} samples to {csv_file}")


def main(args=None):
    rclpy.init(args=args)
    node = SMCControllerNode()
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
