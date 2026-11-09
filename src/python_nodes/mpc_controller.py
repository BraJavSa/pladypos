#!/usr/bin/env python3
"""
mpc_controller.py
=================
Real-Time 30 Hz Smooth Model Predictive Controller (LTV-MPC, N = 15) for the PlaDyPos USV.

Features:
1. Prediction Horizon N = 15 (dt_mpc = 0.12 s -> 1.8 s lookahead) over the 3-DOF
   nonlinear Fossen model linearized around the time-varying Figure-8 reference.
2. Incremental Control Formulation (optimizes d_tau_k = tau_k - tau_{k-1}) with strong
   slew-rate penalty R_delta + output thruster smoothing for ultra-smooth trajectories.
3. Closest-Point Trajectory Initialization: when the first odometry arrives, the Figure-8
   phase t_offset is automatically chosen as the point on the curve closest to the boat's
   current position (x0, y0, psi0), eliminating the initial jump to the pool center.
4. Model-Assisted Velocity Observer: blends 3-DOF Fossen model velocity predictions with
   measured odometry twist so 25 Hz camera differentiation jitter never reaches the thrusters.
5. Publishes the reference Figure-8, closest start point, moving carrot, and executed path
   to RViz on /pool_boundary, /usv5/reference_path, and /usv5/executed_path.
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
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, PoseStamped
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


def wrap_to_pi(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return math.cos(half), 0.0, 0.0, math.sin(half)


# ==============================================================================
# 2. SMOOTH N=15 LTV-MPC SOLVER IN INCREMENTAL FORM
# ==============================================================================
class SmoothFossenMPC:
    """
    3-DOF Smooth LTV Model Predictive Controller (N = 15) in Incremental Form.
    Optimizes control increments d_tau_k = tau_k - tau_{k-1} over N=15 steps
    to guarantee smooth, oscillation-free tracking on the PlaDyPos USV.
    """
    def __init__(self, N: int = 15, dt_mpc: float = 0.12, model: dict = None):
        self.N = int(N)
        self.dt_mpc = float(dt_mpc)
        self.m = (model or DEFAULT_FOSSEN_MODEL).copy()

        # Stage state weights: [e_x_body (m), e_y_body (m), e_psi (rad), e_u (m/s), e_v (m/s), e_r (rad/s)]
        # Tuned for gentle, ripple-free tracking on PlaDyPos (m11~220 kg, m22~230 kg, m33~9.6 kg*m^2)
        self.q_diag = np.array([95000.0, 115000.0, 1800.0, 48000.0, 55000.0, 750.0], dtype=float)
        # Terminal weight (3.0x stage weight for guaranteed asymptotic stability)
        self.q_term = 3.0 * self.q_diag
        # Deviation from feedforward generalized force [tau_u (N), tau_v (N), tau_r (N*m)]
        self.r_tau = np.array([0.25, 0.25, 2.00], dtype=float)
        # Strong slew-rate penalty on control increments d_tau = tau_k - tau_{k-1}
        self.r_delta = np.array([12.0, 12.0, 90.0], dtype=float)

        # Generalized force saturation limits [N, N, N*m]
        self.tau_max = np.array([45.0, 45.0, 15.0], dtype=float)
        # Max change in generalized force per 30 Hz control tick [N, N, N*m]
        self.dtau_tick_max = np.array([2.2, 2.2, 0.75], dtype=float)

        self.tau_prev = np.zeros(3, dtype=float)

        # Lower-triangular summation matrix L (3N x 3N) where U = U_prev + L @ dU
        n_u = 3 * self.N
        self.L = np.zeros((n_u, n_u), dtype=float)
        for i in range(self.N):
            for j in range(i + 1):
                self.L[3*i:3*(i+1), 3*j:3*(j+1)] = np.eye(3)

        self.R_tau_bar = np.diag(np.tile(self.r_tau, self.N))
        self.R_delta_bar = np.diag(np.tile(self.r_delta, self.N))
        q_all = np.concatenate([np.tile(self.q_diag, self.N - 1), self.q_term])
        self.Q_bar = np.diag(q_all)

    def reset(self):
        self.tau_prev = np.zeros(3, dtype=float)

    def _step_matrices(self, u_r: float, v_r: float, r_r: float):
        """Discrete-time (A_d, B_d) of 6-state body-frame tracking error for step dt_mpc."""
        m = self.m
        dt = self.dt_mpc

        du = (m["Xu"] + 2.0 * m["Xuu"] * abs(u_r)) / m["m11"]
        dv = (m["Yv"] + 2.0 * m["Yvv"] * abs(v_r)) / m["m22"]
        dr = (m["Nr"] + 2.0 * m["Nrr"] * abs(r_r)) / m["m33"]

        Ac = np.array([
            [ 0.0,   r_r,  -v_r,  1.0,  0.0,  0.0],
            [-r_r,   0.0,   u_r,  0.0,  1.0,  0.0],
            [ 0.0,   0.0,   0.0,  0.0,  0.0,  1.0],
            [ 0.0,   0.0,   0.0, -du,   r_r * (m["m22"]/m["m11"]),  v_r * (m["m22"]/m["m11"])],
            [ 0.0,   0.0,   0.0, -r_r * (m["m11"]/m["m22"]), -dv, -u_r * (m["m11"]/m["m22"])],
            [ 0.0,   0.0,   0.0,  0.0,  0.0, -dr ]
        ], dtype=float)

        Bc = np.zeros((6, 3), dtype=float)
        Bc[3, 0] = 1.0 / m["m11"]
        Bc[4, 1] = 1.0 / m["m22"]
        Bc[5, 2] = 1.0 / m["m33"]

        I6 = np.eye(6)
        Ad = I6 + dt * Ac + 0.5 * (dt * dt) * (Ac @ Ac)
        Bd = dt * (I6 + 0.5 * dt * Ac) @ Bc
        return Ad, Bd

    def solve(self, eta: np.ndarray, nu: np.ndarray, ref_horizon: list):
        """
        Solves the N=15 LTV-MPC problem at the current state (eta, nu).
        ref_horizon: list of N+1 dicts with keys ('eta_d', 'nu_d', 'tau_ff').
        Returns: (tau_cmd (3,), solve_ms (float))
        """
        t0 = time.perf_counter()
        N = self.N

        eta_d0 = ref_horizon[0]["eta_d"]
        nu_d0 = ref_horizon[0]["nu_d"]

        dx0 = eta[0] - eta_d0[0]
        dy0 = eta[1] - eta_d0[1]
        dpsi0 = wrap_to_pi(eta[2] - eta_d0[2])
        c0 = math.cos(eta[2])
        s0 = math.sin(eta[2])
        ex_b0 = c0 * dx0 + s0 * dy0
        ey_b0 = -s0 * dx0 + c0 * dy0

        # Soft-saturate large position errors so initial convergence is gentle
        max_pos_err = 0.80
        err_norm = math.hypot(ex_b0, ey_b0)
        if err_norm > max_pos_err:
            scale = max_pos_err / err_norm
            ex_b0 *= scale
            ey_b0 *= scale

        e0 = np.array([
            ex_b0,
            ey_b0,
            np.clip(dpsi0, -0.85, 0.85),
            nu[0] - nu_d0[0],
            nu[1] - nu_d0[1],
            nu[2] - nu_d0[2]
        ], dtype=float)

        Ad_list = []
        Bd_list = []
        tau_ff_vec = np.zeros(3 * N, dtype=float)

        for k in range(N):
            nu_dk = ref_horizon[k]["nu_d"]
            Ad_k, Bd_k = self._step_matrices(nu_dk[0], nu_dk[1], nu_dk[2])
            Ad_list.append(Ad_k)
            Bd_list.append(Bd_k)
            tau_ff_vec[3*k:3*(k+1)] = ref_horizon[k]["tau_ff"]

        Sx = np.zeros((6 * N, 6), dtype=float)
        Su = np.zeros((6 * N, 3 * N), dtype=float)

        Phi = np.eye(6)
        for i in range(N):
            Phi = Ad_list[i] @ Phi
            Sx[6*i:6*(i+1), :] = Phi
            Psi = np.eye(6)
            for j in range(i, -1, -1):
                if j < i:
                    Psi = Psi @ Ad_list[j + 1]
                Su[6*i:6*(i+1), 3*j:3*(j+1)] = Psi @ Bd_list[j]

        U_prev_vec = np.tile(self.tau_prev, N)
        d_ff0 = U_prev_vec - tau_ff_vec

        SuL = Su @ self.L
        e_free = (Sx @ e0) + (Su @ d_ff0)

        RL = self.R_tau_bar @ self.L
        H = (SuL.T @ self.Q_bar @ SuL) + (self.L.T @ RL) + self.R_delta_bar
        H = 0.5 * (H + H.T) + 1e-6 * np.eye(3 * N)
        g = (SuL.T @ (self.Q_bar @ e_free)) + (self.L.T @ (self.R_tau_bar @ d_ff0))

        dU = -np.linalg.solve(H, g)

        dtau0 = np.clip(dU[0:3], -self.dtau_tick_max, self.dtau_tick_max)
        tau_cmd = np.clip(self.tau_prev + dtau0, -self.tau_max, self.tau_max)
        self.tau_prev = tau_cmd.copy()

        solve_ms = (time.perf_counter() - t0) * 1000.0
        return tau_cmd, solve_ms


# ==============================================================================
# 3. ROS 2 NODE: MPC CONTROLLER (N=15, CLOSEST-POINT START, SMOOTH OBSERVER)
# ==============================================================================
class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        # Pool dimensions from calibrated camera tags
        self.declare_parameter('center_x', -0.5713)
        self.declare_parameter('center_y', -1.2731)
        self.declare_parameter('span_x', 5.0292)
        self.declare_parameter('span_y', 4.8290)
        self.declare_parameter('scale_factor', 0.70)

        # MPC & Trajectory parameters
        self.declare_parameter('horizon_n', 15)           # N = 15 prediction steps
        self.declare_parameter('dt_mpc', 0.12)            # 0.12 s step -> 1.8 s lookahead
        self.declare_parameter('period_s', 85.0)          # Slower, smoother 85 s period
        self.declare_parameter('control_rate_hz', 30.0)   # 30 Hz real-time control loop
        self.declare_parameter('soft_start_s', 5.0)       # 5.0 s gentle velocity/force ramp
        self.declare_parameter('start_at_closest', True)  # Start at point closest to current pose

        # Smoothing parameters
        self.declare_parameter('vel_filter_alpha', 0.22)  # Blend model-predicted vs odom velocity
        self.declare_parameter('cmd_filter_alpha', 0.28)  # First-order thruster smoothing
        self.declare_parameter('max_cmd_step', 0.05)      # Max thruster delta per 33 ms tick

        # Topics
        self.declare_parameter('odom_topic', '/usv5/odometry/filtered')
        self.declare_parameter('visual_odom_topic', '/usv5/visual_odom')
        self.declare_parameter('imu_topic', '/usv5/imu/data')
        self.declare_parameter('pwm_topic', '/usv5/pwm_out')
        self.declare_parameter('marker_topic', '/pool_boundary')
        self.declare_parameter('frame_id', 'odom')

        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.width_x = float(self.get_parameter('span_x').value)
        self.height_y = float(self.get_parameter('span_y').value)
        self.ratio = float(self.get_parameter('scale_factor').value)

        self.ax_8 = 0.5 * self.width_x * self.ratio
        self.ay_8 = 0.5 * self.height_y * self.ratio

        self.horizon_n = int(self.get_parameter('horizon_n').value)
        self.dt_mpc = float(self.get_parameter('dt_mpc').value)
        self.period_s = float(self.get_parameter('period_s').value)
        self.omega = 2.0 * math.pi / self.period_s
        self.rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.dt = 1.0 / self.rate_hz
        self.soft_start_s = float(self.get_parameter('soft_start_s').value)
        self.start_at_closest = bool(self.get_parameter('start_at_closest').value)

        self.vel_alpha = float(self.get_parameter('vel_filter_alpha').value)
        self.cmd_alpha = float(self.get_parameter('cmd_filter_alpha').value)
        self.max_cmd_step = float(self.get_parameter('max_cmd_step').value)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.visual_odom_topic = self.get_parameter('visual_odom_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.pwm_topic = self.get_parameter('pwm_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.model = DEFAULT_FOSSEN_MODEL.copy()
        self.mpc = SmoothFossenMPC(N=self.horizon_n, dt_mpc=self.dt_mpc, model=self.model)
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

        # Measured vs Model-filtered body velocities (u, v, r)
        self.u_meas = 0.0
        self.v_meas = 0.0
        self.r_meas = 0.0
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0

        # Closest-point phase offset & marker coordinates
        self.start_wall_time = None
        self.t_offset = 0.0
        self.start_x = self.center_x
        self.start_y = self.center_y
        self.cmd_filtered = np.zeros(4, dtype=float)

        self.log_rows = []
        self.exec_poses = []
        self.exec_points = []

        # Precompute full Figure-8 path for RViz visualization
        self.ref_path_msg, self.ref_marker_points = self._build_reference_path_messages()

        # 30 Hz Control & Visualization Timer
        self.create_timer(self.dt, self.control_loop_tick)
        # 1 Hz Static Reference Marker Publisher
        self.create_timer(1.0, self.publish_reference_visuals)

        self.get_logger().info(
            f"Smooth 3-DOF Fossen MPC Controller (N={self.horizon_n}) initialized at {self.rate_hz:.0f} Hz:\n"
            f"  Pool Center (X_c, Y_c) : ({self.center_x:.4f}, {self.center_y:.4f}) m\n"
            f"  70% Figure-8 Semi-Axes : A_x = {self.ax_8:.4f} m, A_y = {self.ay_8:.4f} m\n"
            f"  Figure-8 Period        : {self.period_s:.1f} s (dt_mpc = {self.dt_mpc:.2f} s)\n"
            f"  Start at Closest Point : {self.start_at_closest}\n"
            f"  PWM Output Topic       : {self.pwm_topic}"
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

    def evaluate_figure8_ref(self, t_sec: float, s_ramp: float = 1.0):
        """Evaluates reference pose eta_d, body velocity nu_d, and Fossen feedforward tau_ff."""
        w = self.omega
        Ax, Ay = self.ax_8, self.ay_8
        xc, yc = self.center_x, self.center_y

        xd = xc + Ax * math.sin(w * t_sec)
        yd = yc + Ay * math.sin(2.0 * w * t_sec)

        dxd_raw = Ax * w * math.cos(w * t_sec)
        dyd_raw = 2.0 * Ay * w * math.cos(2.0 * w * t_sec)

        ddxd_raw = -Ax * (w ** 2) * math.sin(w * t_sec)
        ddyd_raw = -4.0 * Ay * (w ** 2) * math.sin(2.0 * w * t_sec)

        dddxd = -Ax * (w ** 3) * math.cos(w * t_sec)
        dddyd = -8.0 * Ay * (w ** 3) * math.cos(2.0 * w * t_sec)

        psid = math.atan2(dyd_raw, dxd_raw)
        v2 = dxd_raw * dxd_raw + dyd_raw * dyd_raw + 1e-12
        dpsid_raw = (dxd_raw * ddyd_raw - dyd_raw * ddxd_raw) / v2
        dv2_dt = 2.0 * (dxd_raw * ddxd_raw + dyd_raw * ddyd_raw)
        ddpsid_raw = ((dxd_raw * dddyd - dyd_raw * dddxd) * v2 - (dxd_raw * ddyd_raw - dyd_raw * ddxd_raw) * dv2_dt) / (v2 * v2)

        dxd = s_ramp * dxd_raw
        dyd = s_ramp * dyd_raw
        dpsid = s_ramp * dpsid_raw

        ddxd = (s_ramp ** 2) * ddxd_raw
        ddyd = (s_ramp ** 2) * ddyd_raw
        ddpsid = (s_ramp ** 2) * ddpsid_raw

        c = math.cos(psid)
        s = math.sin(psid)
        ud = c * dxd + s * dyd
        vd = -s * dxd + c * dyd
        rd = dpsid

        dud = c * ddxd + s * ddyd + vd * rd
        dvd = -s * ddxd + c * ddyd - ud * rd
        drd = ddpsid

        m = self.model
        tau_ff = np.array([
            m["m11"] * dud - m["m22"] * vd * rd + (m["Xu"] + m["Xuu"] * abs(ud)) * ud,
            m["m22"] * dvd + m["m11"] * ud * rd + (m["Yv"] + m["Yvv"] * abs(vd)) * vd,
            m["m33"] * drd + (m["m22"] - m["m11"]) * ud * vd + (m["Nr"] + m["Nrr"] * abs(rd)) * rd
        ], dtype=float)

        eta_d = np.array([xd, yd, psid], dtype=float)
        nu_d = np.array([ud, vd, rd], dtype=float)
        return {"eta_d": eta_d, "nu_d": nu_d, "tau_ff": tau_ff}

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
            # Primarily minimize Euclidean distance, breaking ties at the center crossing via yaw
            score = d2 + (0.25 ** 2) * (1.0 - math.cos(dpsi))
            if score < best_score:
                best_score = score
                best_t = t_cand

        return best_t

    def _build_reference_path_messages(self):
        stamp = self.get_clock().now().to_msg()
        path_msg = NavPath()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.frame_id

        pts = []
        n_pts = 360
        for i in range(n_pts + 1):
            t_i = (i / n_pts) * self.period_s
            ref_i = self.evaluate_figure8_ref(t_i)
            eta_i = ref_i["eta_d"]
            qw, qx, qy, qz = yaw_to_quat(float(eta_i[2]))

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

        # 2. Closest Trajectory Start Point Marker (Green Sphere at (start_x, start_y))
        start_marker = Marker()
        start_marker.header.stamp = stamp
        start_marker.header.frame_id = self.frame_id
        start_marker.ns = "figure8_start"
        start_marker.id = 11
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = float(self.start_x)
        start_marker.pose.position.y = float(self.start_y)
        start_marker.pose.position.z = 0.05
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.18
        start_marker.scale.y = 0.18
        start_marker.scale.z = 0.18
        start_marker.color = ColorRGBA(r=0.06, g=0.72, b=0.51, a=0.95)  # Emerald Green
        self.marker_pub.publish(start_marker)

    def imu_cb(self, msg: Imu):
        self.r_meas = float(msg.angular_velocity.z)
        self.has_imu_r = True

    def visual_odom_cb(self, msg: Odometry):
        self.vx_odom = float(msg.twist.twist.linear.x)
        self.vy_odom = -float(msg.twist.twist.linear.y)
        self.has_visual_vel = True

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

        if abs(u_ekf) > 1e-9 or abs(v_ekf) > 1e-9 or abs(r_ekf) > 1e-9:
            self.u_meas = float(np.clip(u_ekf, -1.2, 1.2))
            self.v_meas = float(np.clip(v_ekf, -1.2, 1.2))
            if not self.has_imu_r:
                self.r_meas = float(np.clip(r_ekf, -2.0, 2.0))
        else:
            if self.prev_odom_time is not None:
                dt_odom = max(1e-3, now_s - self.prev_odom_time)
                if not self.has_visual_vel:
                    vx_raw = (x_new - self.prev_x) / dt_odom
                    vy_raw = (y_new - self.prev_y) / dt_odom
                    self.vx_odom = 0.7 * self.vx_odom + 0.3 * vx_raw
                    self.vy_odom = 0.7 * self.vy_odom + 0.3 * vy_raw
                if not self.has_imu_r:
                    dyaw = wrap_to_pi(yaw_new - self.prev_yaw)
                    self.r_meas = 0.7 * self.r_meas + 0.3 * (dyaw / dt_odom)
            cos_y = math.cos(yaw_new)
            sin_y = math.sin(yaw_new)
            self.u_meas = float(np.clip(self.vx_odom * cos_y + self.vy_odom * sin_y, -1.2, 1.2))
            self.v_meas = float(np.clip(-self.vx_odom * sin_y + self.vy_odom * cos_y, -1.2, 1.2))

        self.x = x_new
        self.y = y_new
        self.yaw = yaw_new
        self.prev_x = x_new
        self.prev_y = y_new
        self.prev_yaw = yaw_new
        self.prev_odom_time = now_s
        self.has_odom = True

    def _update_velocity_observer(self):
        """
        Model-Assisted Velocity Observer:
        Predicts (u, v, r) one step forward using the 3-DOF Fossen model and the
        previously commanded generalized force tau_prev, then blends with measured
        odometry twist so camera differentiation jitter is completely filtered out.
        """
        m = self.model
        tau_applied = self.mpc.tau_prev
        u, v, r = self.u, self.v, self.r

        du = (tau_applied[0] + m["m22"] * v * r - (m["Xu"] + m["Xuu"] * abs(u)) * u) / m["m11"]
        dv = (tau_applied[1] - m["m11"] * u * r - (m["Yv"] + m["Yvv"] * abs(v)) * v) / m["m22"]
        dr = (tau_applied[2] - (m["m22"] - m["m11"]) * u * v - (m["Nr"] + m["Nrr"] * abs(r)) * r) / m["m33"]

        u_pred = u + self.dt * du
        v_pred = v + self.dt * dv
        r_pred = r + self.dt * dr

        a = self.vel_alpha
        self.u = (1.0 - a) * u_pred + a * self.u_meas
        self.v = (1.0 - a) * v_pred + a * self.v_meas
        self.r = (1.0 - a) * r_pred + a * self.r_meas

    def control_loop_tick(self):
        if not self.has_odom:
            return

        now_s = time.monotonic()
        if self.start_wall_time is None:
            self.start_wall_time = now_s
            self.u = self.u_meas
            self.v = self.v_meas
            self.r = self.r_meas
            if self.start_at_closest:
                self.t_offset = self.find_closest_trajectory_time(self.x, self.y, self.yaw)
            else:
                self.t_offset = 0.0
            ref0 = self.evaluate_figure8_ref(self.t_offset, s_ramp=1.0)
            self.start_x = float(ref0["eta_d"][0])
            self.start_y = float(ref0["eta_d"][1])
            dist0 = math.hypot(self.x - self.start_x, self.y - self.start_y)
            self.publish_reference_visuals()
            self.get_logger().info(
                f"First odometry at ({self.x:.2f}, {self.y:.2f}, yaw={math.degrees(self.yaw):.1f} deg).\n"
                f"  -> Starting Figure-8 at closest phase t_offset = {self.t_offset:.2f} s "
                f"({self.start_x:.2f}, {self.start_y:.2f}) [initial distance = {dist0:.2f} m]."
            )

        t_elapsed = now_s - self.start_wall_time

        # Smooth cosine ramp over soft_start_s seconds
        if self.soft_start_s > 0.0 and t_elapsed < self.soft_start_s:
            s_ramp = 0.5 * (1.0 - math.cos(math.pi * t_elapsed / self.soft_start_s))
        else:
            s_ramp = 1.0

        # Update model-assisted velocity observer
        self._update_velocity_observer()

        # Build N=15 prediction horizon along the Figure-8 starting from t_offset + t_elapsed
        t_traj = self.t_offset + t_elapsed
        horizon = []
        for h in range(self.horizon_n + 1):
            t_h = t_traj + h * self.dt_mpc
            horizon.append(self.evaluate_figure8_ref(t_h, s_ramp=s_ramp))

        eta = np.array([self.x, self.y, self.yaw], dtype=float)
        nu = np.array([self.u, self.v, self.r], dtype=float)

        # Solve N=15 Smooth LTV-MPC
        tau_cmd, solve_ms = self.mpc.solve(eta, nu, horizon)

        # Thruster allocation & Richards inverse mapping -> [-1.0, 1.0]
        f_motors = PHI_PINV @ tau_cmd
        cmd_raw = thrust_to_cmd_richards(f_motors)

        # Rate-limit & low-pass filter thruster commands for gentle hardware actuation
        cmd_target = (1.0 - self.cmd_alpha) * self.cmd_filtered + self.cmd_alpha * cmd_raw
        delta_cmd = np.clip(cmd_target - self.cmd_filtered, -self.max_cmd_step, self.max_cmd_step)
        self.cmd_filtered = np.clip(self.cmd_filtered + delta_cmd, -1.0, 1.0)

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

        # Publish real-time target carrot and executed trajectory to RViz
        eta_d0 = horizon[0]["eta_d"]
        nu_d0 = horizon[0]["nu_d"]
        self._publish_realtime_rviz(eta_d0)

        # Log sample
        self.log_rows.append([
            f"{t_elapsed:.4f}",
            f"{eta_d0[0]:.5f}", f"{eta_d0[1]:.5f}", f"{eta_d0[2]:.5f}",
            f"{self.x:.5f}", f"{self.y:.5f}", f"{self.yaw:.5f}",
            f"{nu_d0[0]:.5f}", f"{nu_d0[1]:.5f}", f"{nu_d0[2]:.5f}",
            f"{self.u:.5f}", f"{self.v:.5f}", f"{self.r:.5f}",
            f"{tau_cmd[0]:.4f}", f"{tau_cmd[1]:.4f}", f"{tau_cmd[2]:.4f}",
            f"{out_data[0]:.4f}", f"{out_data[1]:.4f}", f"{out_data[2]:.4f}", f"{out_data[3]:.4f}",
            f"{solve_ms:.3f}"
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
        if len(self.exec_points) > 2600:
            self.exec_points = self.exec_points[-2600:]

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
        if len(self.exec_poses) > 2600:
            self.exec_poses = self.exec_poses[-2600:]

        exec_path_msg = NavPath()
        exec_path_msg.header.stamp = stamp
        exec_path_msg.header.frame_id = self.frame_id
        exec_path_msg.poses = self.exec_poses
        self.exec_path_pub.publish(exec_path_msg)

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
            csv_file = out_dir / f"realtime_mpc_N15_{ts}.csv"
            latest_file = out_dir / "realtime_mpc_N15_30Hz.csv"
            header = [
                "t",
                "x_ref", "y_ref", "psi_ref",
                "x_real", "y_real", "psi_real",
                "u_ref", "v_ref", "r_ref",
                "u_real", "v_real", "r_real",
                "Tu", "Tv", "Tr",
                "cmd_m1_FR", "cmd_m2_FL", "cmd_m3_BR", "cmd_m4_BL",
                "solve_ms"
            ]
            for target_path in (csv_file, latest_file):
                with open(target_path, "w", newline="", encoding="utf-8") as fp:
                    writer = csv.writer(fp)
                    writer.writerow(header)
                    writer.writerows(self.log_rows)
            self.get_logger().info(f"Saved {len(self.log_rows)} MPC samples to {latest_file}")


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
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
