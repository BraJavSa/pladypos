#!/usr/bin/env python3
"""
Standalone 3-DOF Inverse Dynamics (Computed Torque) Trajectory Tracking Simulation
for the PlaDyPos USV at 30 Hz (No ROS communication).

Features:
  - Uses the 9-parameter white-box Fossen model identified at 30 Hz from
    `identified_models_pladypos.json`.
  - Computes the exact pool bounding box from `all_experiments.csv`:
      X in [-3.0859, 1.9433] m  -> Center X_c = -0.5713 m, Span = 5.0292 m
      Y in [-3.6876, 1.1414] m  -> Center Y_c = -1.2731 m, Span = 4.8290 m
  - Generates a smooth Figure-8 (Lemniscate of Gerono) reference trajectory
    centered at (X_c, Y_c) — NOT at (0, 0) of odom — occupying exactly 70%
    of the pool area (A_x = 0.35 * Span_X, A_y = 0.35 * Span_Y).
  - Implements full 3-DOF Fossen Inverse Dynamics (Feedback Linearization /
    Computed Torque) with exact kinematic acceleration decoupling at 30 Hz.
  - Saves simulation data to `inverse_dynamics_figure8_30Hz.csv` and generates
    an academic IEEE-style English PDF report `inverse_dynamics_figure8_report.pdf`.
"""

import csv
import json
from pathlib import Path
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from matplotlib.patches import Rectangle

# ==============================================================================
# ACADEMIC / IEEE SERIF STYLING (matching min-jerk-flatness-planning)
# ==============================================================================
plt.rcParams.update({
    'font.size': 10.0,
    'axes.labelsize': 10.0,
    'xtick.labelsize': 8.0,
    'ytick.labelsize': 8.0,
    'legend.fontsize': 7.5,
    'font.family': 'serif',
    'mathtext.fontset': 'cm',
    'figure.facecolor': 'white',
    'axes.facecolor': 'white',
    'savefig.facecolor': 'white',
})

COLOR_REAL = '#000000'      # Solid/Dashed Black for Simulated USV Response
COLOR_DESIRED = '#B22222'   # Crimson Red for Desired Reference
COLOR_NAVY = '#003366'      # Navy Blue
COLOR_GREEN = '#2E8B57'     # Sea Green
COLOR_AMBER = '#D97706'     # Amber
COLOR_GRAY = '#6B7280'      # Neutral Gray


# ==============================================================================
# 1. POOL GEOMETRY & 70% CENTERED FIGURE-8 REFERENCE TRAJECTORY (AT 30 HZ)
# ==============================================================================
def get_pool_bounds(data_dir: Path) -> dict:
    """Computes the pool bounding box and geometric center from all_experiments.csv."""
    csv_path = data_dir / "all_experiments.csv"
    if csv_path.exists():
        raw = np.genfromtxt(csv_path, delimiter=',', names=True, dtype=None, encoding='utf-8')
        x_all = np.asarray(raw['x'], dtype=float)
        y_all = np.asarray(raw['y'], dtype=float)
        x_min, x_max = float(np.min(x_all)), float(np.max(x_all))
        y_min, y_max = float(np.min(y_all)), float(np.max(y_all))
    else:
        x_min, x_max = -3.0859, 1.9433
        y_min, y_max = -3.6876, 1.1414

    x_center = 0.5 * (x_min + x_max)
    y_center = 0.5 * (y_min + y_max)
    span_x = x_max - x_min
    span_y = y_max - y_min

    return {
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "x_center": x_center,
        "y_center": y_center,
        "span_x": span_x,
        "span_y": span_y,
        "scale_factor": 0.70,
        "ax_70": 0.70 * 0.5 * span_x,
        "ay_70": 0.70 * 0.5 * span_y,
    }


def generate_figure8_reference(t: np.ndarray, pool: dict, period_s: float = 55.0):
    """
    Generates a smooth Figure-8 (Lissajous / Lemniscate) trajectory centered at
    (x_center, y_center) occupying 70% of the pool width and height:
        x_d(t) = x_c + A_x * sin(w * t)
        y_d(t) = y_c + A_y * sin(2 * w * t)
    with tangent heading psi_d(t) = atan2(dy_d, dx_d) and exact 1st & 2nd derivatives.
    """
    xc = pool["x_center"]
    yc = pool["y_center"]
    Ax = pool["ax_70"]
    Ay = pool["ay_70"]
    w = 2.0 * np.pi / period_s

    # Positions
    xd = xc + Ax * np.sin(w * t)
    yd = yc + Ay * np.sin(2.0 * w * t)

    # 1st inertial derivatives
    dxd = Ax * w * np.cos(w * t)
    dyd = 2.0 * Ay * w * np.cos(2.0 * w * t)

    # 2nd inertial derivatives
    ddxd = -Ax * (w ** 2) * np.sin(w * t)
    ddyd = -4.0 * Ay * (w ** 2) * np.sin(2.0 * w * t)

    # 3rd inertial derivatives (for exact angular acceleration ddpsi_d)
    dddxd = -Ax * (w ** 3) * np.cos(w * t)
    dddyd = -8.0 * Ay * (w ** 3) * np.cos(2.0 * w * t)

    # Tangent heading psi_d and its exact derivatives
    psid = np.unwrap(np.arctan2(dyd, dxd))
    v2 = dxd ** 2 + dyd ** 2 + 1e-12
    dpsid = (dxd * ddyd - dyd * ddxd) / v2
    dv2_dt = 2.0 * (dxd * ddxd + dyd * ddyd)
    ddpsid = ((dxd * dddyd - dyd * dddxd) * v2 - (dxd * ddyd - dyd * ddxd) * dv2_dt) / (v2 ** 2)

    eta_d = np.column_stack([xd, yd, psid])
    deta_d = np.column_stack([dxd, dyd, dpsid])
    ddeta_d = np.column_stack([ddxd, ddyd, ddpsid])

    # Desired body-frame velocities nu_d = J(psi_d)^T * deta_d
    cos_p = np.cos(psid)
    sin_p = np.sin(psid)
    ud = cos_p * dxd + sin_p * dyd
    vd = -sin_p * dxd + cos_p * dyd
    rd = dpsid
    nu_d = np.column_stack([ud, vd, rd])

    return eta_d, deta_d, ddeta_d, nu_d


# ==============================================================================
# 2. 3-DOF FOSSEN MODEL, ALLOCATION MATRIX & INVERSE DYNAMICS CONTROLLER
# ==============================================================================
D_ARM = 0.65 / np.sqrt(2.0)             # D = 0.65 / sqrt(2) ~= 0.459619 m
COS45 = np.cos(np.deg2rad(45.0))        # cos(45 deg) ~= 0.707107
SIN45 = np.sin(np.deg2rad(45.0))        # sin(45 deg) ~= 0.707107

# Exact thruster allocation matrix used in system identification (identify_fossen_3dof.py)
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


def cmd_to_thrust_richards(cmd: np.ndarray) -> np.ndarray:
    """Computes thrust T [N] from normalized command cmd in [-1, 1] using the Richards curve."""
    cmd_s = np.clip(np.asarray(cmd, dtype=float), -1.0, 1.0)
    T = np.zeros_like(cmd_s, dtype=float)
    p = RICHARDS_PARAMS
    pos_mask = cmd_s > p['deadzone']
    neg_mask = cmd_s < -p['deadzone']

    if np.any(pos_mask):
        c_pos = cmd_s[pos_mask]
        denom_pos = (p['C_pos'] + np.exp(-p['B_pos'] * (c_pos - p['M_pos']))) ** (1.0 / p['v_pos'])
        T_pos = p['A_pos'] + (p['K_pos'] - p['A_pos']) / denom_pos
        T[pos_mask] = np.clip(T_pos, 0.0, p['maxForceFwd'])

    if np.any(neg_mask):
        c_neg = cmd_s[neg_mask]
        denom_neg = (p['C_neg'] + np.exp(-p['B_neg'] * (c_neg - p['M_neg']))) ** (1.0 / p['v_neg'])
        T_neg = p['A_neg'] + (p['K_neg'] - p['A_neg']) / denom_neg
        T[neg_mask] = np.clip(T_neg, p['maxForceRev'], 0.0)

    return T


def thrust_to_cmd_richards(T_des: np.ndarray) -> np.ndarray:
    """Inverts the Richards curve to map desired thruster force T [N] to normalized command cmd in [-1, 1]."""
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
            c = p['M_pos'] - np.log(inner) / p['B_pos']
            cmd_out.flat[i] = float(np.clip(c, p['deadzone'], 1.0))
        else:
            f_c = float(np.clip(f_val, p['maxForceRev'] + 1e-3, -0.05))
            ratio = (p['K_neg'] - p['A_neg']) / (f_c - p['A_neg'])
            inner = max(1e-9, (ratio ** p['v_neg']) - p['C_neg'])
            c = p['M_neg'] - np.log(inner) / p['B_neg']
            cmd_out.flat[i] = float(np.clip(c, -1.0, -p['deadzone']))

    return cmd_out


def wrap_to_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def rotation_matrix_3dof(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=float)


def fossen_coriolis_damping(nu: np.ndarray, model: dict):
    """Computes C(nu)*nu and D(nu)*nu for the 9-parameter 3-DOF Fossen model."""
    u, v, r = float(nu[0]), float(nu[1]), float(nu[2])
    m11, m22, m33 = model["m11"], model["m22"], model["m33"]
    Xu, Xuu = model["Xu"], model["Xuu"]
    Yv, Yvv = model["Yv"], model["Yvv"]
    Nr, Nrr = model["Nr"], model["Nrr"]

    C_nu = np.array([
        -m22 * v * r,
         m11 * u * r,
        (m22 - m11) * u * v
    ], dtype=float)

    D_nu = np.array([
        (Xu + Xuu * abs(u)) * u,
        (Yv + Yvv * abs(v)) * v,
        (Nr + Nrr * abs(r)) * r
    ], dtype=float)

    return C_nu, D_nu


def inverse_dynamics_control(
    eta: np.ndarray,
    nu: np.ndarray,
    eta_d: np.ndarray,
    deta_d: np.ndarray,
    ddeta_d: np.ndarray,
    model: dict,
    Kp: np.ndarray,
    Kd: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    3-DOF Inverse Dynamics (Computed Torque / Feedback Linearization) Controller:
      1. Outer kinematic PD + feedforward acceleration in inertial frame:
           a_eta = ddeta_d + Kd * (deta_d - deta) + Kp * e_eta
      2. Exact mapping to body-frame acceleration command:
           a_nu = J(psi)^T * (a_eta - dJ(psi, r) * nu)
                = J(psi)^T * a_eta - S(r) * nu
      3. Nonlinear Fossen inverse dynamics compensation:
           tau_cmd = M * a_nu + C(nu)*nu + D(nu)*nu
    """
    psi = float(eta[2])
    u, v, r = float(nu[0]), float(nu[1]), float(nu[2])

    J = rotation_matrix_3dof(psi)
    deta = J @ nu

    # Pose and velocity tracking errors in inertial frame
    e_eta = np.array([
        eta_d[0] - eta[0],
        eta_d[1] - eta[1],
        wrap_to_pi(eta_d[2] - eta[2])
    ], dtype=float)
    de_eta = deta_d - deta

    # Virtual control acceleration in inertial frame
    a_eta = ddeta_d + Kd @ de_eta + Kp @ e_eta

    # Exact kinematic transformation: dJ(psi, r) * nu = J(psi) * [-v*r, u*r, 0]^T
    s_nu = np.array([-v * r, u * r, 0.0], dtype=float)
    a_nu = J.T @ a_eta - s_nu

    # Fossen dynamic matrices M, C(nu)*nu, D(nu)*nu
    M_diag = np.array([model["m11"], model["m22"], model["m33"]], dtype=float)
    C_nu, D_nu = fossen_coriolis_damping(nu, model)

    # Computed torque control law
    tau_cmd = M_diag * a_nu + C_nu + D_nu

    # Allocate generalized forces to the 4 X-configuration thrusters [M1:FR, M2:FL, M3:BR, M4:BL]
    f_motors = PHI_PINV @ tau_cmd
    cmd_norm = thrust_to_cmd_richards(f_motors)
    f_motors_sat = cmd_to_thrust_richards(cmd_norm)
    tau_applied = PHI_ALLOC @ f_motors_sat
    pwm_us = 1500.0 + 400.0 * cmd_norm

    return tau_applied, tau_cmd, pwm_us


def rk4_step_fossen(eta: np.ndarray, nu: np.ndarray, tau: np.ndarray, dt: float, model: dict):
    """Integrates the 6-state 3-DOF Fossen USV model [eta, nu] over one step dt using RK4."""
    M_inv = np.array([1.0 / model["m11"], 1.0 / model["m22"], 1.0 / model["m33"]], dtype=float)

    def dyn(state):
        eta_s = state[:3]
        nu_s = state[3:]
        J_s = rotation_matrix_3dof(eta_s[2])
        deta_s = J_s @ nu_s
        C_s, D_s = fossen_coriolis_damping(nu_s, model)
        dnu_s = M_inv * (tau - C_s - D_s)
        return np.concatenate([deta_s, dnu_s])

    x0 = np.concatenate([eta, nu])
    k1 = dyn(x0)
    k2 = dyn(x0 + 0.5 * dt * k1)
    k3 = dyn(x0 + 0.5 * dt * k2)
    k4 = dyn(x0 + dt * k3)
    x_next = x0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return x_next[:3], x_next[3:]


# ==============================================================================
# 3. SIMULATION & ACADEMIC PDF PLOTTING
# ==============================================================================
def run_simulation():
    data_dir = Path("/home/brayan/ros2_ws/src/pladypos/experiments/processed_data")
    json_path = data_dir / "identified_models_pladypos.json"

    with open(json_path, "r", encoding="utf-8") as fp:
        models_data = json.load(fp)
    model = models_data["stage2_whitebox_refined_optimal_9param"]

    pool = get_pool_bounds(data_dir)

    # 30 Hz sampling rate (matching system identification)
    fs = 30.0
    dt = 1.0 / fs
    period_s = 55.0
    t_final = period_s
    t = np.arange(0.0, t_final + 0.5 * dt, dt)
    N = len(t)

    eta_d, deta_d, ddeta_d, nu_d = generate_figure8_reference(t, pool, period_s=period_s)

    # Critically damped / well-tuned PD gains for Inverse Dynamics outer loop:
    # wn_pos = 1.5 rad/s, zeta_pos = 1.0 -> Kp = 2.25, Kd = 3.0
    # wn_yaw = 2.2 rad/s, zeta_yaw = 1.0 -> Kp = 4.84, Kd = 4.4
    Kp = np.diag([2.25, 2.25, 4.84])
    Kd = np.diag([3.00, 3.00, 4.40])

    # State arrays
    eta_sim = np.zeros((N, 3), dtype=float)
    nu_sim = np.zeros((N, 3), dtype=float)
    tau_sim = np.zeros((N, 3), dtype=float)
    pwm_sim = np.zeros((N, 4), dtype=float)

    # Initial state: start at the initial point of the centered figure-8
    eta_sim[0] = eta_d[0].copy()
    nu_sim[0] = nu_d[0].copy()

    for k in range(N - 1):
        tau_app, _, pwm_k = inverse_dynamics_control(
            eta_sim[k], nu_sim[k], eta_d[k], deta_d[k], ddeta_d[k], model, Kp, Kd
        )
        tau_sim[k] = tau_app
        pwm_sim[k] = pwm_k
        eta_sim[k + 1], nu_sim[k + 1] = rk4_step_fossen(eta_sim[k], nu_sim[k], tau_app, dt, model)

    # Final sample control input
    tau_sim[-1], _, pwm_sim[-1] = inverse_dynamics_control(
        eta_sim[-1], nu_sim[-1], eta_d[-1], deta_d[-1], ddeta_d[-1], model, Kp, Kd
    )

    # Save simulation results to CSV
    out_csv = data_dir / "inverse_dynamics_figure8_30Hz.csv"
    with open(out_csv, "w", newline="", encoding="utf-8") as fp:
        writer = csv.writer(fp)
        writer.writerow([
            "t",
            "x_ref", "y_ref", "psi_ref",
            "x_sim", "y_sim", "psi_sim",
            "u_ref", "v_ref", "r_ref",
            "u_sim", "v_sim", "r_sim",
            "Tu", "Tv", "Tr",
            "pwm_m1_FR", "pwm_m2_FL", "pwm_m3_BR", "pwm_m4_BL"
        ])
        for k in range(N):
            writer.writerow([
                f"{t[k]:.4f}",
                f"{eta_d[k, 0]:.5f}", f"{eta_d[k, 1]:.5f}", f"{eta_d[k, 2]:.5f}",
                f"{eta_sim[k, 0]:.5f}", f"{eta_sim[k, 1]:.5f}", f"{eta_sim[k, 2]:.5f}",
                f"{nu_d[k, 0]:.5f}", f"{nu_d[k, 1]:.5f}", f"{nu_d[k, 2]:.5f}",
                f"{nu_sim[k, 0]:.5f}", f"{nu_sim[k, 1]:.5f}", f"{nu_sim[k, 2]:.5f}",
                f"{tau_sim[k, 0]:.4f}", f"{tau_sim[k, 1]:.4f}", f"{tau_sim[k, 2]:.4f}",
                f"{pwm_sim[k, 0]:.1f}", f"{pwm_sim[k, 1]:.1f}", f"{pwm_sim[k, 2]:.1f}", f"{pwm_sim[k, 3]:.1f}",
            ])

    # Compute tracking metrics
    err_pos = np.hypot(eta_sim[:, 0] - eta_d[:, 0], eta_sim[:, 1] - eta_d[:, 1])
    rmse_pos = float(np.sqrt(np.mean(err_pos ** 2)))
    rmse_u = float(np.sqrt(np.mean((nu_sim[:, 0] - nu_d[:, 0]) ** 2)))
    rmse_v = float(np.sqrt(np.mean((nu_sim[:, 1] - nu_d[:, 1]) ** 2)))
    rmse_r = float(np.sqrt(np.mean((nu_sim[:, 2] - nu_d[:, 2]) ** 2)))

    # Generate Academic IEEE PDF Report
    out_pdf = data_dir / "inverse_dynamics_figure8_report.pdf"
    out_png = data_dir / "inverse_dynamics_figure8_report.png"

    fig, axes = plt.subplots(3, 2, figsize=(10.0, 8.4), dpi=300)

    # 1. Row 0, Col 0: 2D Pool Area, 70% Centered Region & Figure-8 Trajectory
    ax = axes[0, 0]
    # Full pool boundary (100% of experimental workspace)
    pool_rect = Rectangle(
        (pool["x_min"], pool["y_min"]),
        pool["span_x"],
        pool["span_y"],
        fill=False,
        edgecolor=COLOR_GRAY,
        linestyle=':',
        linewidth=1.2,
        label='Pool Boundary (100%)',
        zorder=1,
    )
    ax.add_patch(pool_rect)

    # 70% centered operating box
    box70_rect = Rectangle(
        (pool["x_center"] - pool["ax_70"], pool["y_center"] - pool["ay_70"]),
        2.0 * pool["ax_70"],
        2.0 * pool["ay_70"],
        fill=True,
        facecolor='#F8FAFC',
        edgecolor=COLOR_NAVY,
        linestyle='-.',
        linewidth=1.0,
        alpha=0.8,
        label='Centered 70% Workspace',
        zorder=2,
    )
    ax.add_patch(box70_rect)

    ax.plot(eta_d[:, 0], eta_d[:, 1], color=COLOR_DESIRED, linestyle='-', linewidth=1.8, label='Desired Figure-8', zorder=3)
    ax.plot(eta_sim[:, 0], eta_sim[:, 1], color=COLOR_REAL, linestyle='--', linewidth=1.3, label='Inverse Dynamics', zorder=4)
    ax.scatter(pool["x_center"], pool["y_center"], color=COLOR_NAVY, s=40, marker='+', linewidths=1.4, label='Pool Center', zorder=5)
    ax.scatter(eta_sim[0, 0], eta_sim[0, 1], color='#10B981', s=45, marker='o', edgecolors='black', linewidths=0.8, label='Start', zorder=6)

    ax.set_xlabel(r'Position $X \ [\mathrm{m}]$')
    ax.set_ylabel(r'Position $Y \ [\mathrm{m}]$')
    ax.set_xlim(pool["x_min"] - 0.35, pool["x_max"] + 0.35)
    ax.set_ylim(pool["y_min"] - 0.35, pool["y_max"] + 1.15)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper center', ncol=2, frameon=True, edgecolor='black', fontsize=6.5)
    ax.xaxis.set_major_formatter(FormatStrFormatter('%.1f'))
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    # 2. Row 0, Col 1: Inertial Positions X(t), Y(t) and Heading psi(t)
    ax = axes[0, 1]
    ax.plot(t, eta_d[:, 0], color=COLOR_DESIRED, linestyle='-', linewidth=1.6, label=r'Desired $X_d$', zorder=2)
    ax.plot(t, eta_sim[:, 0], color=COLOR_REAL, linestyle='--', linewidth=1.2, label=r'Simulated $X$', zorder=3)
    ax.plot(t, eta_d[:, 1], color=COLOR_NAVY, linestyle='-', linewidth=1.6, label=r'Desired $Y_d$', zorder=2)
    ax.plot(t, eta_sim[:, 1], color=COLOR_GREEN, linestyle='-.', linewidth=1.2, label=r'Simulated $Y$', zorder=3)
    ax.set_xlabel(r'Time $t \ [\mathrm{s}]$')
    ax.set_ylabel(r'Position $X, Y \ [\mathrm{m}]$')
    ax.set_xlim(0, t[-1])
    ax.set_ylim(pool["y_min"] - 0.2, pool["x_max"] + 0.6)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right', ncol=2, frameon=True, edgecolor='black', fontsize=7.0)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    # 3. Row 1, Col 0: Surge Velocity u(t)
    ax = axes[1, 0]
    ax.plot(t, nu_d[:, 0], color=COLOR_DESIRED, linestyle='-', linewidth=1.7, label='Desired', zorder=2)
    ax.plot(t, nu_sim[:, 0], color=COLOR_REAL, linestyle='--', linewidth=1.2, label='Inverse Dynamics', zorder=3)
    ax.set_xlabel(r'Time $t \ [\mathrm{s}]$')
    ax.set_ylabel(r'Surge Velocity $u \ [\mathrm{m/s}]$')
    ax.set_xlim(0, t[-1])
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right', frameon=True, edgecolor='black', fontsize=7.5)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

    # 4. Row 1, Col 1: Sway Velocity v(t)
    ax = axes[1, 1]
    ax.plot(t, nu_d[:, 1], color=COLOR_DESIRED, linestyle='-', linewidth=1.7, label='Desired', zorder=2)
    ax.plot(t, nu_sim[:, 1], color=COLOR_REAL, linestyle='--', linewidth=1.2, label='Inverse Dynamics', zorder=3)
    ax.set_xlabel(r'Time $t \ [\mathrm{s}]$')
    ax.set_ylabel(r'Sway Velocity $v \ [\mathrm{m/s}]$')
    ax.set_xlim(0, t[-1])
    ax.set_ylim(-0.10, 0.10)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right', frameon=True, edgecolor='black', fontsize=7.5)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

    # 5. Row 2, Col 0: Yaw Rate r(t)
    ax = axes[2, 0]
    ax.plot(t, nu_d[:, 2], color=COLOR_DESIRED, linestyle='-', linewidth=1.7, label='Desired', zorder=2)
    ax.plot(t, nu_sim[:, 2], color=COLOR_REAL, linestyle='--', linewidth=1.2, label='Inverse Dynamics', zorder=3)
    ax.set_xlabel(r'Time $t \ [\mathrm{s}]$')
    ax.set_ylabel(r'Yaw Rate $r \ [\mathrm{rad/s}]$')
    ax.set_xlim(0, t[-1])
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right', frameon=True, edgecolor='black', fontsize=7.5)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

    # 6. Row 2, Col 1: Generalized Control Actions tau = [T_u, T_v, T_r]^T
    ax = axes[2, 1]
    ax.plot(t, tau_sim[:, 0], color=COLOR_NAVY, linestyle='-', linewidth=1.3, label=r'$T_u \ [\mathrm{N}]$', zorder=3)
    ax.plot(t, tau_sim[:, 1], color=COLOR_GREEN, linestyle='-.', linewidth=1.3, label=r'$T_v \ [\mathrm{N}]$', zorder=3)
    ax.plot(t, tau_sim[:, 2], color=COLOR_DESIRED, linestyle='--', linewidth=1.3, label=r'$T_r \ [\mathrm{N\cdot m}]$', zorder=4)
    ax.set_xlabel(r'Time $t \ [\mathrm{s}]$')
    ax.set_ylabel(r'Generalized Inputs $\tau$')
    ax.set_xlim(0, t[-1])
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right', ncol=3, frameon=True, edgecolor='black', fontsize=7.0)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    plt.tight_layout()
    fig.savefig(out_pdf, format='pdf', dpi=300, bbox_inches='tight')
    fig.savefig(out_png, format='png', dpi=300, bbox_inches='tight')
    plt.close(fig)

    print("======================================================================")
    print("  3-DOF INVERSE DYNAMICS SIMULATION (FIGURE-8 AT 30 HZ)")
    print("======================================================================")
    print(f"  Pool X range (100%)   : [{pool['x_min']:.4f}, {pool['x_max']:.4f}] m  (Span = {pool['span_x']:.4f} m)")
    print(f"  Pool Y range (100%)   : [{pool['y_min']:.4f}, {pool['y_max']:.4f}] m  (Span = {pool['span_y']:.4f} m)")
    print(f"  Pool Geometric Center : (X_c, Y_c) = ({pool['x_center']:.4f}, {pool['y_center']:.4f}) m")
    print(f"  70% Figure-8 Semi-Axes: A_x = {pool['ax_70']:.4f} m, A_y = {pool['ay_70']:.4f} m")
    print(f"  70% X bounds          : [{pool['x_center'] - pool['ax_70']:.4f}, {pool['x_center'] + pool['ax_70']:.4f}] m")
    print(f"  70% Y bounds          : [{pool['y_center'] - pool['ay_70']:.4f}, {pool['y_center'] + pool['ay_70']:.4f}] m")
    print("----------------------------------------------------------------------")
    print(f"  Position RMSE         : {rmse_pos:.6f} m")
    print(f"  Velocity RMSE (u,v,r) : u={rmse_u:.6f} m/s, v={rmse_v:.6f} m/s, r={rmse_r:.6f} rad/s")
    print(f"  Peak Control Inputs   : |Tu|_max={np.max(np.abs(tau_sim[:,0])):.2f} N, "
          f"|Tv|_max={np.max(np.abs(tau_sim[:,1])):.2f} N, |Tr|_max={np.max(np.abs(tau_sim[:,2])):.2f} N*m")
    print(f"  CSV saved to          : {out_csv}")
    print(f"  PDF saved to          : {out_pdf}")
    print("======================================================================")


if __name__ == "__main__":
    run_simulation()
