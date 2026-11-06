#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3-DOF Fossen Dynamic Model Identification and Open-Loop Validation for PlaDyPos USV
====================================================================================
Two-stage white-box identification:
  - Stage 1: Constrained Linear Least Squares (lsq_linear) on the Fossen equation of motion.
  - Stage 2: White-box nonlinear simulation refinement (least_squares) of the 9 physical
    Fossen parameters [m11, m22, m33, Xu, Xuu, Yv, Yvv, Nr, Nrr].
Generates an IEEE/academic-styled multi-page PDF report matching min-jerk-flatness-planning.
"""

import json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter
from scipy.optimize import lsq_linear, least_squares

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import FormatStrFormatter

# ==============================================================================
# ACADEMIC / IEEE PLOT STYLING (from min-jerk-flatness-planning)
# ==============================================================================
plt.rcParams.update({
    'font.size': 10.0,
    'axes.labelsize': 10.0,
    'xtick.labelsize': 8.0,
    'ytick.labelsize': 8.0,
    'font.family': 'serif',
    'mathtext.fontset': 'cm',
    'figure.facecolor': 'white',
})

COLOR_REAL = '#000000'
COLOR_MODEL = '#B22222'
COLOR_OPENLOOP = '#2563EB'
COLOR_NAVY = '#003366'
COLOR_GREEN = '#2E8B57'
COLOR_AMBER = '#D97706'
COLOR_GRAY = '#6B7280'

# ==============================================================================
# 1. PHYSICAL PARAMETERS & RICHARDS THRUSTER MODEL (T200)
# ==============================================================================
MASS_NOMINAL = 25.0                     # m [kg]
D_ARM = 0.65 / np.sqrt(2.0)             # D = 0.65 / sqrt(2) ~= 0.459619 m
COS45 = np.cos(np.deg2rad(45.0))        # cos(45 deg) ~= 0.707107
SIN45 = np.sin(np.deg2rad(45.0))        # sin(45 deg) ~= 0.707107

# Thruster allocation matrix Phi (3x4):
# Order: tau_i = [tau_1 (M1:FR), tau_2 (M2:FL), tau_3 (M3:BR), tau_4 (M4:BL)]^T
PHI_ALLOC = np.array([
    [ COS45,  COS45, -COS45, -COS45],
    [ SIN45, -SIN45,  SIN45, -SIN45],
    [ D_ARM, -D_ARM, -D_ARM,  D_ARM]
], dtype=float)

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


def pwm_to_cmd(pwm_us: np.ndarray, pwm_neutral: float = 1500.0, pwm_scale: float = 400.0) -> np.ndarray:
    """Converts PWM in microseconds to normalized command cmd saturated in [-1, 1]."""
    return np.clip((pwm_us - pwm_neutral) / pwm_scale, -1.0, 1.0)


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


def compute_generalized_forces(pwm_m1: np.ndarray, pwm_m2: np.ndarray,
                               pwm_m3: np.ndarray, pwm_m4: np.ndarray):
    """Converts the 4 PWM signals (us) to cmd in [-1, 1] and computes tau = Phi * tau_i."""
    cmd1 = pwm_to_cmd(pwm_m1)
    cmd2 = pwm_to_cmd(pwm_m2)
    cmd3 = pwm_to_cmd(pwm_m3)
    cmd4 = pwm_to_cmd(pwm_m4)

    tau1 = cmd_to_thrust_richards(cmd1)
    tau2 = cmd_to_thrust_richards(cmd2)
    tau3 = cmd_to_thrust_richards(cmd3)
    tau4 = cmd_to_thrust_richards(cmd4)

    tau_i = np.vstack([tau1, tau2, tau3, tau4])  # (4, N)
    tau = PHI_ALLOC @ tau_i                      # (3, N) -> [T_u, T_v, T_r]
    cmds = np.vstack([cmd1, cmd2, cmd3, cmd4])   # (4, N)
    return cmds, tau_i, tau


# ==============================================================================
# 2. 3-DOF FOSSEN DYNAMICS AND OPEN-LOOP KINEMATICS (RK4)
# ==============================================================================
def simulate_fossen_velocities(t: np.ndarray, tau: np.ndarray, nu0: np.ndarray, params: np.ndarray) -> np.ndarray:
    """
    Integrates the 3-DOF Fossen body-frame dynamics via RK4:
        M * dot(nu) + C(nu) * nu + D(nu) * nu = tau
    where params = [m11, m22, m33, X_u, X_uu, Y_v, Y_vv, N_r, N_rr].
    """
    m11, m22, m33, X_u, X_uu, Y_v, Y_vv, N_r, N_rr = params
    N = len(t)
    nu_sim = np.zeros((N, 3), dtype=float)
    nu_sim[0] = nu0

    u, v, r = float(nu0[0]), float(nu0[1]), float(nu0[2])

    def fossen_rhs(u_s: float, v_s: float, r_s: float, Tu: float, Tv: float, Tr: float):
        du = (Tu + m22 * v_s * r_s - (X_u + X_uu * abs(u_s)) * u_s) / m11
        dv = (Tv - m11 * u_s * r_s - (Y_v + Y_vv * abs(v_s)) * v_s) / m22
        dr = (Tr - (m22 - m11) * u_s * v_s - (N_r + N_rr * abs(r_s)) * r_s) / m33
        return du, dv, dr

    for k in range(N - 1):
        dt = float(t[k + 1] - t[k])
        if dt <= 0.0:
            nu_sim[k + 1] = [u, v, r]
            continue

        Tu0, Tv0, Tr0 = float(tau[0, k]), float(tau[1, k]), float(tau[2, k])
        Tu1, Tv1, Tr1 = float(tau[0, k + 1]), float(tau[1, k + 1]), float(tau[2, k + 1])
        Tum, Tvm, Trm = 0.5 * (Tu0 + Tu1), 0.5 * (Tv0 + Tv1), 0.5 * (Tr0 + Tr1)

        k1_u, k1_v, k1_r = fossen_rhs(u, v, r, Tu0, Tv0, Tr0)
        k2_u, k2_v, k2_r = fossen_rhs(u + 0.5 * dt * k1_u, v + 0.5 * dt * k1_v, r + 0.5 * dt * k1_r, Tum, Tvm, Trm)
        k3_u, k3_v, k3_r = fossen_rhs(u + 0.5 * dt * k2_u, v + 0.5 * dt * k2_v, r + 0.5 * dt * k2_r, Tum, Tvm, Trm)
        k4_u, k4_v, k4_r = fossen_rhs(u + dt * k3_u, v + dt * k3_v, r + dt * k3_r, Tu1, Tv1, Tr1)

        u += (dt / 6.0) * (k1_u + 2.0 * k2_u + 2.0 * k3_u + k4_u)
        v += (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v)
        r += (dt / 6.0) * (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r)

        u = float(np.clip(u, -5.0, 5.0))
        v = float(np.clip(v, -5.0, 5.0))
        r = float(np.clip(r, -10.0, 10.0))

        nu_sim[k + 1] = [u, v, r]

    return nu_sim


def integrate_fossen_kinematics(t: np.ndarray, nu_mod: np.ndarray, eta0: np.ndarray,
                                psi_meas: np.ndarray = None):
    """Integrates dot(eta) = J(psi) * nu in standard open-loop."""
    N = len(t)
    eta_full = np.zeros((N, 3), dtype=float)
    eta_full[0] = eta0

    eta_kin = np.zeros((N, 3), dtype=float)
    eta_kin[0] = eta0

    x_f, y_f, psi_f = float(eta0[0]), float(eta0[1]), float(eta0[2])
    x_k, y_k = float(eta0[0]), float(eta0[1])

    for k in range(N - 1):
        dt = float(t[k + 1] - t[k])
        if dt <= 0.0:
            eta_full[k + 1] = [x_f, y_f, psi_f]
            eta_kin[k + 1] = [x_k, y_k, psi_f]
            continue

        u0, v0, r0 = nu_mod[k]
        u1, v1, r1 = nu_mod[k + 1]
        um, vm, rm = 0.5 * (u0 + u1), 0.5 * (v0 + v1), 0.5 * (r0 + r1)

        psi_next = psi_f + dt * rm
        psi_mid = 0.5 * (psi_f + psi_next)

        dx_f = um * np.cos(psi_mid) - vm * np.sin(psi_mid)
        dy_f = um * np.sin(psi_mid) + vm * np.cos(psi_mid)
        x_f += dt * dx_f
        y_f += dt * dy_f
        psi_f = psi_next
        eta_full[k + 1] = [x_f, y_f, psi_f]

        if psi_meas is not None:
            psi_m_mid = 0.5 * (psi_meas[k] + psi_meas[k + 1])
        else:
            psi_m_mid = psi_mid
        dx_k = um * np.cos(psi_m_mid) - vm * np.sin(psi_m_mid)
        dy_k = um * np.sin(psi_m_mid) + vm * np.cos(psi_m_mid)
        x_k += dt * dx_k
        y_k += dt * dy_k
        eta_kin[k + 1] = [x_k, y_k, psi_f]

    return eta_full, eta_kin


# ==============================================================================
# 3. TWO-STAGE WHITE-BOX IDENTIFICATION
# ==============================================================================
def despike_signal(sig: np.ndarray, window: int = 5, threshold: float = 0.25) -> np.ndarray:
    """Removes isolated single-sample glitches (like clean_outliers in plot_realtime_results.py)."""
    out = np.asarray(sig, dtype=float).copy()
    n = len(out)
    half = window // 2
    for i in range(half, n - half):
        med = np.median(out[i - half:i + half + 1])
        if abs(out[i] - med) > threshold:
            out[i] = med
    return out


def load_experiments(data_dir: Path):
    """Loads the 6 experiment CSV files."""
    exp_files = sorted(data_dir.glob("exp_*.csv"))
    datasets = []
    for f in exp_files:
        raw = np.genfromtxt(f, delimiter=',', names=True, dtype=None, encoding='utf-8')
        t = np.asarray(raw['t'], dtype=float)
        x = np.asarray(raw['x'], dtype=float)
        y = np.asarray(raw['y'], dtype=float)
        psi = np.asarray(raw['yaw'], dtype=float)
        psi_deg = np.asarray(raw['yaw_deg'], dtype=float)
        u = despike_signal(np.asarray(raw['u'], dtype=float), window=5, threshold=0.22)
        v = despike_signal(np.asarray(raw['v'], dtype=float), window=5, threshold=0.22)
        r = despike_signal(np.asarray(raw['r'], dtype=float), window=5, threshold=0.60)

        pwm1 = np.asarray(raw['pwm_us_m1_FR'], dtype=float)
        pwm2 = np.asarray(raw['pwm_us_m2_FL'], dtype=float)
        pwm3 = np.asarray(raw['pwm_us_m3_BR'], dtype=float)
        pwm4 = np.asarray(raw['pwm_us_m4_BL'], dtype=float)

        cmds, tau_i, tau = compute_generalized_forces(pwm1, pwm2, pwm3, pwm4)

        datasets.append({
            'name': f.stem,
            'path': f,
            't': t,
            'x': x,
            'y': y,
            'psi': psi,
            'psi_deg': psi_deg,
            'u': u,
            'v': v,
            'r': r,
            'nu_meas': np.column_stack([u, v, r]),
            'eta_meas': np.column_stack([x, y, psi]),
            'pwm': np.vstack([pwm1, pwm2, pwm3, pwm4]),
            'cmds': cmds,
            'tau_i': tau_i,
            'tau': tau
        })
    return datasets


def identify_stage1_linear_ls(datasets):
    """Stage 1: Direct Constrained Linear Least Squares (lsq_linear)."""
    Phi9_list, Tau_list = [], []
    Phi6_list = []
    Phi5_list = []

    for d in datasets:
        t = d['t']
        dt_med = float(np.median(np.diff(t)))
        u_s = savgol_filter(d['u'], 15, 3)
        v_s = savgol_filter(d['v'], 15, 3)
        r_s = savgol_filter(d['r'], 15, 3)
        du = savgol_filter(d['u'], 15, 3, deriv=1, delta=dt_med)
        dv = savgol_filter(d['v'], 15, 3, deriv=1, delta=dt_med)
        dr = savgol_filter(d['r'], 15, 3, deriv=1, delta=dt_med)
        Tu, Tv, Tr = d['tau'][0], d['tau'][1], d['tau'][2]

        for k in range(len(t)):
            Phi9_list.append([du[k], -v_s[k]*r_s[k], 0.0, u_s[k], abs(u_s[k])*u_s[k], 0.0, 0.0, 0.0, 0.0])
            Tau_list.append(Tu[k])
            Phi9_list.append([u_s[k]*r_s[k], dv[k], 0.0, 0.0, 0.0, v_s[k], abs(v_s[k])*v_s[k], 0.0, 0.0])
            Tau_list.append(Tv[k])
            Phi9_list.append([-u_s[k]*v_s[k], u_s[k]*v_s[k], dr[k], 0.0, 0.0, 0.0, 0.0, r_s[k], abs(r_s[k])*r_s[k]])
            Tau_list.append(Tr[k])

            Phi6_list.append([du[k], -v_s[k]*r_s[k], 0.0, u_s[k], 0.0, 0.0])
            Phi6_list.append([u_s[k]*r_s[k], dv[k], 0.0, 0.0, v_s[k], 0.0])
            Phi6_list.append([-u_s[k]*v_s[k], u_s[k]*v_s[k], dr[k], 0.0, 0.0, r_s[k]])

            Phi5_list.append([du[k] - v_s[k]*r_s[k], 0.0, u_s[k], 0.0, 0.0])
            Phi5_list.append([dv[k] + u_s[k]*r_s[k], 0.0, 0.0, v_s[k], 0.0])
            Phi5_list.append([0.0, dr[k], 0.0, 0.0, r_s[k]])

    Phi9 = np.asarray(Phi9_list, dtype=float)
    Phi6 = np.asarray(Phi6_list, dtype=float)
    Phi5 = np.asarray(Phi5_list, dtype=float)
    Tau = np.asarray(Tau_list, dtype=float)

    lb9 = [25.0, 25.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ub9 = [100.0, 100.0, 20.0, 250.0, 250.0, 250.0, 250.0, 100.0, 100.0]
    theta9_ls = lsq_linear(Phi9, Tau, bounds=(lb9, ub9)).x

    lb6 = [25.0, 25.0, 2.0, 0.0, 0.0, 0.0]
    ub6 = [100.0, 100.0, 20.0, 250.0, 250.0, 100.0]
    t6 = lsq_linear(Phi6, Tau, bounds=(lb6, ub6)).x
    theta6_ls = np.array([t6[0], t6[1], t6[2], t6[3], 0.0, t6[4], 0.0, t6[5], 0.0])

    lb5 = [25.0, 2.0, 0.0, 0.0, 0.0]
    ub5 = [100.0, 20.0, 250.0, 250.0, 100.0]
    t5 = lsq_linear(Phi5, Tau, bounds=(lb5, ub5)).x
    theta5_ls = np.array([t5[0], t5[0], t5[1], t5[2], 0.0, t5[3], 0.0, t5[4], 0.0])

    return theta9_ls, theta6_ls, theta5_ls


def refine_stage2_whitebox(datasets, theta9_ls):
    """
    Stage 2: White-box parameter refinement (theta_final = theta_MC + Delta_theta)
    optimizing open-loop velocities (u, v, r) and open-loop trajectory (x, y, psi).
    """
    w_u = 1.0 / 0.11
    w_v = 1.0 / 0.11
    w_r = 1.0 / 0.26
    w_psi = 0.18 / 0.50

    def sim_residuals(p):
        res_list = []
        for d in datasets:
            nu_sim = simulate_fossen_velocities(d['t'], d['tau'], d['nu_meas'][0], p)
            err_u = (nu_sim[:, 0] - d['u']) * w_u
            err_v = (nu_sim[:, 1] - d['v']) * w_v
            err_r = (nu_sim[:, 2] - d['r']) * w_r
            dt = np.diff(d['t'], prepend=d['t'][0])
            psi_sim = d['psi'][0] + np.cumsum(0.5 * (nu_sim[:, 2] + np.roll(nu_sim[:, 2], 1)) * dt)
            psi_sim[0] = d['psi'][0]
            err_psi = (psi_sim - d['psi']) * w_psi
            res_list.append(np.concatenate([err_u, err_v, err_r, err_psi]))
        return np.concatenate(res_list)

    lb_phys = np.array([25.0, 25.0, 2.0, 1.0, 0.0, 1.0, 0.0, 0.2, 0.0])
    ub_phys = np.array([100.0, 100.0, 20.0, 300.0, 300.0, 300.0, 300.0, 100.0, 100.0])
    opt_phys = least_squares(sim_residuals, theta9_ls, bounds=(lb_phys, ub_phys), loss='soft_l1', max_nfev=2000)
    theta9_phys = opt_phys.x

    lb_free = np.array([25.0, 25.0, 0.5, 1.0, 0.0, 1.0, 0.0, 0.2, 0.0])
    ub_free = np.array([450.0, 450.0, 80.0, 500.0, 400.0, 500.0, 400.0, 100.0, 100.0])
    opt_free = least_squares(sim_residuals, theta9_phys, bounds=(lb_free, ub_free), loss='soft_l1', max_nfev=2500)
    theta9_opt = opt_free.x

    def sim_residuals_6(p6):
        p_full = np.array([p6[0], p6[1], p6[2], p6[3], 0.0, p6[4], 0.0, p6[5], 0.0])
        return sim_residuals(p_full)

    p6_0 = np.array([theta9_opt[0], theta9_opt[1], theta9_opt[2],
                     theta9_opt[3] + 20.0, theta9_opt[5] + 15.0, theta9_opt[7] + 1.5])
    opt_6 = least_squares(
        sim_residuals_6, p6_0,
        bounds=([25.0, 25.0, 0.5, 1.0, 1.0, 0.2], [450.0, 450.0, 80.0, 500.0, 500.0, 100.0]),
        loss='soft_l1', max_nfev=1500
    )
    t6 = opt_6.x
    theta6_opt = np.array([t6[0], t6[1], t6[2], t6[3], 0.0, t6[4], 0.0, t6[5], 0.0])

    return theta9_phys, theta9_opt, theta6_opt


def compute_metrics(y_true: np.ndarray, y_pred: np.ndarray):
    err = y_pred - y_true
    rmse = float(np.sqrt(np.mean(err ** 2)))
    mae = float(np.mean(np.abs(err)))
    ss_res = float(np.sum(err ** 2))
    ss_tot = float(np.sum((y_true - np.mean(y_true)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    if np.std(y_true) > 1e-9 and np.std(y_pred) > 1e-9:
        corr = float(np.corrcoef(y_true, y_pred)[0, 1])
    else:
        corr = 0.0
    return {'rmse': rmse, 'mae': mae, 'r2': r2, 'corr': corr}


def vec_to_dict(theta: np.ndarray):
    return {
        "m11": float(theta[0]),
        "m22": float(theta[1]),
        "m33": float(theta[2]),
        "Xu": float(theta[3]),
        "Xuu": float(theta[4]),
        "Yv": float(theta[5]),
        "Yvv": float(theta[6]),
        "Nr": float(theta[7]),
        "Nrr": float(theta[8]),
    }


def save_parameter_files(out_dir: Path, theta9_ls, theta9_phys, theta9_opt, theta6_opt, theta5_ls):
    json_path = out_dir / "identified_models_pladypos.json"
    models_json = {
        "stage1_linear_least_squares_9param": vec_to_dict(theta9_ls),
        "stage2_whitebox_refined_bounded_9param": vec_to_dict(theta9_phys),
        "stage2_whitebox_refined_optimal_9param": vec_to_dict(theta9_opt),
        "stage2_whitebox_delta_bounded": vec_to_dict(theta9_phys - theta9_ls),
        "stage2_whitebox_delta_optimal": vec_to_dict(theta9_opt - theta9_ls),
        "linear_6_parameters_refined": vec_to_dict(theta6_opt),
        "symmetric_5_parameters_ls": vec_to_dict(theta5_ls),
    }
    with open(json_path, "w", encoding="utf-8") as fp:
        json.dump(models_json, fp, indent=4)

    param_csv = out_dir / "fossen_3dof_parameters.csv"
    names = ["m11", "m22", "m33", "Xu", "Xuu", "Yv", "Yvv", "Nr", "Nrr"]
    symbols = ["m - X_du", "m - Y_dv", "Iz - N_dr", "X_u", "X_uu", "Y_v", "Y_vv", "N_r", "N_rr"]
    units = ["kg", "kg", "kg*m^2", "N*s/m", "N*s^2/m^2", "N*s/m", "N*s^2/m^2", "N*m*s/rad", "N*m*s^2/rad^2"]
    with open(param_csv, "w", encoding="utf-8") as f:
        f.write("parameter,symbol,units,stage1_linear_LS,delta_bounded,stage2_bounded_final,delta_optimal,stage2_optimal_final,linear_6param\n")
        for i in range(9):
            f.write(
                f"{names[i]},{symbols[i]},{units[i]},"
                f"{theta9_ls[i]:.6f},{theta9_phys[i]-theta9_ls[i]:.6f},{theta9_phys[i]:.6f},"
                f"{theta9_opt[i]-theta9_ls[i]:.6f},{theta9_opt[i]:.6f},{theta6_opt[i]:.6f}\n"
            )

    param_yaml = out_dir / "fossen_3dof_parameters.yaml"
    with open(param_yaml, "w", encoding="utf-8") as f:
        f.write("# White-Box 3-DOF Fossen Models Identified for PlaDyPos\n")
        f.write("pladypos_fossen_3dof:\n")
        f.write("  stage1_linear_least_squares:\n")
        for k, v in vec_to_dict(theta9_ls).items():
            f.write(f"    {k}: {v:.6f}\n")
        f.write("  stage2_whitebox_refined_bounded:\n")
        for k, v in vec_to_dict(theta9_phys).items():
            f.write(f"    {k}: {v:.6f}\n")
        f.write("  stage2_whitebox_refined_optimal:\n")
        for k, v in vec_to_dict(theta9_opt).items():
            f.write(f"    {k}: {v:.6f}\n")
        f.write("  linear_6_parameters:\n")
        for k, v in vec_to_dict(theta6_opt).items():
            f.write(f"    {k}: {v:.6f}\n")

    return json_path, param_csv, param_yaml


# ==============================================================================
# 4. ACADEMIC / IEEE PDF GENERATION (NO NUMBERING, NO TITLES, NO BANNERS)
# ==============================================================================
def generate_pdf_report(pdf_path: Path, datasets):
    """
    Generates a clean, publication-grade multi-page PDF matching the aesthetic of
    min-jerk-flatness-planning:
      - Serif Computer Modern font ('serif' + 'cm')
      - No figure title, no subplot titles, no numbering, no parameter banner
      - All axis labels and legends in English
      - Only the 3 body-frame velocities (u, v, r) and control actions (tau)
    """
    with PdfPages(pdf_path) as pdf:
        for d in datasets:
            fig, axes = plt.subplots(2, 2, figsize=(10.0, 5.6), dpi=300)

            t = d['t']
            nu_sim = d['nu_opt']

            # Row 0, Col 0: Surge Velocity u
            ax = axes[0, 0]
            ax.plot(t, d['u'], color=COLOR_REAL, linestyle='-', linewidth=1.2, label='Real', zorder=3)
            ax.plot(t, nu_sim[:, 0], color=COLOR_MODEL, linestyle='--', linewidth=1.4, label='Modeled', zorder=4)
            ax.set_xlabel(r'Time $t \ [\mathrm{s}]$')
            ax.set_ylabel(r'Surge Velocity $u \ [\mathrm{m/s}]$')
            ax.set_xlim(0, t[-1])
            ax.grid(True, linestyle='--', alpha=0.6)
            ax.legend(loc='upper right', frameon=True, edgecolor='black', fontsize=7.5)
            ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

            # Row 0, Col 1: Sway Velocity v
            ax = axes[0, 1]
            ax.plot(t, d['v'], color=COLOR_REAL, linestyle='-', linewidth=1.2, label='Real', zorder=3)
            ax.plot(t, nu_sim[:, 1], color=COLOR_MODEL, linestyle='--', linewidth=1.4, label='Modeled', zorder=4)
            ax.set_xlabel(r'Time $t \ [\mathrm{s}]$')
            ax.set_ylabel(r'Sway Velocity $v \ [\mathrm{m/s}]$')
            ax.set_xlim(0, t[-1])
            ax.grid(True, linestyle='--', alpha=0.6)
            ax.legend(loc='upper right', frameon=True, edgecolor='black', fontsize=7.5)
            ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

            # Row 1, Col 0: Yaw Rate r
            ax = axes[1, 0]
            ax.plot(t, d['r'], color=COLOR_REAL, linestyle='-', linewidth=1.2, label='Real', zorder=3)
            ax.plot(t, nu_sim[:, 2], color=COLOR_MODEL, linestyle='--', linewidth=1.4, label='Modeled', zorder=4)
            ax.set_xlabel(r'Time $t \ [\mathrm{s}]$')
            ax.set_ylabel(r'Yaw Rate $r \ [\mathrm{rad/s}]$')
            ax.set_xlim(0, t[-1])
            ax.grid(True, linestyle='--', alpha=0.6)
            ax.legend(loc='upper right', frameon=True, edgecolor='black', fontsize=7.5)
            ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

            # Row 1, Col 1: Generalized Control Inputs (T_u, T_v, T_r)
            ax = axes[1, 1]
            ax.plot(t, d['tau'][0], color=COLOR_NAVY, linestyle='-', linewidth=1.2, label=r'$T_u \ [\mathrm{N}]$', zorder=3)
            ax.plot(t, d['tau'][1], color=COLOR_GREEN, linestyle='-.', linewidth=1.2, label=r'$T_v \ [\mathrm{N}]$', zorder=3)
            ax.plot(t, d['tau'][2], color=COLOR_MODEL, linestyle='--', linewidth=1.2, label=r'$T_r \ [\mathrm{N\cdot m}]$', zorder=4)
            ax.set_xlabel(r'Time $t \ [\mathrm{s}]$')
            ax.set_ylabel(r'Generalized Inputs $\tau$')
            ax.set_xlim(0, t[-1])
            ax.grid(True, linestyle='--', alpha=0.6)
            ax.legend(loc='upper right', ncol=3, frameon=True, edgecolor='black', fontsize=7.2)
            ax.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

            plt.tight_layout()
            pdf.savefig(fig, dpi=300, bbox_inches='tight')
            plt.close(fig)


def dict_to_vec(d: dict) -> np.ndarray:
    return np.array([d["m11"], d["m22"], d["m33"], d["Xu"], d["Xuu"], d["Yv"], d["Yvv"], d["Nr"], d["Nrr"]], dtype=float)


def main():
    data_dir = Path("/home/brayan/ros2_ws/src/pladypos/experiments/processed_data")
    datasets = load_experiments(data_dir)
    json_path = data_dir / "identified_models_pladypos.json"

    if json_path.exists():
        with open(json_path, "r", encoding="utf-8") as fp:
            saved = json.load(fp)
        theta9_ls = dict_to_vec(saved["stage1_linear_least_squares_9param"])
        theta9_phys = dict_to_vec(saved["stage2_whitebox_refined_bounded_9param"])
        theta9_opt = dict_to_vec(saved["stage2_whitebox_refined_optimal_9param"])
        theta6_opt = dict_to_vec(saved["linear_6_parameters_refined"])
        theta5_ls = dict_to_vec(saved["symmetric_5_parameters_ls"])
    else:
        theta9_ls, theta6_ls, theta5_ls = identify_stage1_linear_ls(datasets)
        theta9_phys, theta9_opt, theta6_opt = refine_stage2_whitebox(datasets, theta9_ls)
        save_parameter_files(data_dir, theta9_ls, theta9_phys, theta9_opt, theta6_opt, theta5_ls)

    for d in datasets:
        d['nu_opt'] = simulate_fossen_velocities(d['t'], d['tau'], d['nu_meas'][0], theta9_opt)
        d['eta_full_opt'], d['eta_kin_opt'] = integrate_fossen_kinematics(d['t'], d['nu_opt'], d['eta_meas'][0], psi_meas=d['psi'])

    pdf_path = data_dir / "fossen_3dof_identification_report.pdf"
    generate_pdf_report(pdf_path, datasets)
    print(f"[Success] Academic PDF report saved to: {pdf_path}")


if __name__ == "__main__":
    main()
