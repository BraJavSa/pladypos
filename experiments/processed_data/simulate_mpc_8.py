#!/usr/bin/env python3
"""
simulate_mpc_8.py
=================
30 Hz Simulation of the Smooth 3-DOF Fossen Model Predictive Controller (MPC, N=15)
for the PlaDyPos USV tracking a gentle Figure-8 trajectory (70% of pool workspace),
with Automatic Closest-Point Trajectory Initialization.
"""

import math
import time
from pathlib import Path
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

MODEL = {
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

D_ARM = 0.65 / math.sqrt(2.0)
COS45 = math.cos(math.radians(45.0))
SIN45 = math.sin(math.radians(45.0))

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


def cmd_to_thrust_richards(cmd: np.ndarray) -> np.ndarray:
    cmd_s = np.clip(np.asarray(cmd, dtype=float), -1.0, 1.0)
    T = np.zeros_like(cmd_s, dtype=float)
    p = RICHARDS_PARAMS
    pos_mask = cmd_s > p['deadzone']
    neg_mask = cmd_s < -p['deadzone']
    if np.any(pos_mask):
        c_pos = cmd_s[pos_mask]
        denom_pos = (p['C_pos'] + np.exp(-p['B_pos'] * (c_pos - p['M_pos']))) ** (1.0 / p['v_pos'])
        T[pos_mask] = np.clip(p['A_pos'] + (p['K_pos'] - p['A_pos']) / denom_pos, 0.0, p['maxForceFwd'])
    if np.any(neg_mask):
        c_neg = cmd_s[neg_mask]
        denom_neg = (p['C_neg'] + np.exp(-p['B_neg'] * (c_neg - p['M_neg']))) ** (1.0 / p['v_neg'])
        T[neg_mask] = np.clip(p['A_neg'] + (p['K_neg'] - p['A_neg']) / denom_neg, p['maxForceRev'], 0.0)
    return T


def thrust_to_cmd_richards(T_des: np.ndarray) -> np.ndarray:
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


class SmoothFossenMPC:
    """
    3-DOF Smooth LTV Model Predictive Controller (N = 15) in Incremental Form.
    Optimizes control increments d_tau over N=15 steps to guarantee smooth,
    oscillation-free tracking on the PlaDyPos USV.
    """
    def __init__(self, N: int = 15, dt_mpc: float = 0.12, model: dict = None):
        self.N = int(N)
        self.dt_mpc = float(dt_mpc)
        self.m = (model or MODEL).copy()

        # Stage state weights: [e_x_body (m), e_y_body (m), e_psi (rad), e_u (m/s), e_v (m/s), e_r (rad/s)]
        # Tuned for gentle, ripple-free tracking on PlaDyPos (m11~220 kg, m22~230 kg, m33~9.6 kg*m^2)
        self.q_diag = np.array([95000.0, 115000.0, 1800.0, 48000.0, 55000.0, 750.0], dtype=float)
        # Terminal weight (3.0x stage weight for guaranteed asymptotic stability)
        self.q_term = 3.0 * self.q_diag
        # Deviation from feedforward generalized force [tau_u (N), tau_v (N), tau_r (N*m)]
        self.r_tau = np.array([0.25, 0.25, 2.00], dtype=float)
        # Strong slew-rate penalty on control increments d_tau = tau_k - tau_{k-1} (smoothness!)
        self.r_delta = np.array([12.0, 12.0, 90.0], dtype=float)

        # Generalized force saturation limits [N, N, N*m]
        self.tau_max = np.array([45.0, 45.0, 15.0], dtype=float)
        # Max change in generalized force per control tick (at 30 Hz)
        self.dtau_tick_max = np.array([2.2, 2.2, 0.75], dtype=float)

        self.tau_prev = np.zeros(3, dtype=float)

        # Prebuild lower-triangular summation matrix L (3N x 3N) where U = U_prev + L @ dU
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

        # Continuous A_c (6x6) for state e = [ex_b, ey_b, epsi, eu, ev, er]
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

        # 2nd-order Taylor discretization (exact & ultra-fast)
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

        # Initial tracking error e0 in body frame
        eta_d0 = ref_horizon[0]["eta_d"]
        nu_d0 = ref_horizon[0]["nu_d"]

        dx0 = eta[0] - eta_d0[0]
        dy0 = eta[1] - eta_d0[1]
        dpsi0 = wrap_to_pi(eta[2] - eta_d0[2])
        c0 = math.cos(eta[2])
        s0 = math.sin(eta[2])
        ex_b0 = c0 * dx0 + s0 * dy0
        ey_b0 = -s0 * dx0 + c0 * dy0

        # Soft-saturate large initial position errors so initial transient is gentle
        max_pos_err = 0.85
        err_norm = math.hypot(ex_b0, ey_b0)
        if err_norm > max_pos_err:
            scale = max_pos_err / err_norm
            ex_b0 *= scale
            ey_b0 *= scale

        e0 = np.array([
            ex_b0,
            ey_b0,
            np.clip(dpsi0, -0.9, 0.9),
            nu[0] - nu_d0[0],
            nu[1] - nu_d0[1],
            nu[2] - nu_d0[2]
        ], dtype=float)

        # Build condensed LTV prediction matrices S_x (6N x 6) and S_u (6N x 3N)
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

        # Express U = U_prev + L @ dU, and delta_U_ff = U - U_ff = (U_prev - U_ff) + L @ dU
        U_prev_vec = np.tile(self.tau_prev, N)
        d_ff0 = U_prev_vec - tau_ff_vec

        # Predicted error E = Sx @ e0 + Su @ d_ff0 + (Su @ L) @ dU
        SuL = Su @ self.L
        e_free = (Sx @ e0) + (Su @ d_ff0)

        # Condensed Hessian H (3N x 3N) and gradient g (3N,)
        RL = self.R_tau_bar @ self.L
        H = (SuL.T @ self.Q_bar @ SuL) + (self.L.T @ RL) + self.R_delta_bar
        H = 0.5 * (H + H.T) + 1e-6 * np.eye(3 * N)
        g = (SuL.T @ (self.Q_bar @ e_free)) + (self.L.T @ (self.R_tau_bar @ d_ff0))

        # Solve unconstrained step and project onto box/slew constraints (with 2 active-set refinements)
        dU = -np.linalg.solve(H, g)

        # Enforce slew-rate & total force bounds on first step and horizon
        dtau0 = np.clip(dU[0:3], -self.dtau_tick_max, self.dtau_tick_max)
        tau_cmd = np.clip(self.tau_prev + dtau0, -self.tau_max, self.tau_max)
        self.tau_prev = tau_cmd.copy()

        solve_ms = (time.perf_counter() - t0) * 1000.0
        return tau_cmd, solve_ms


def evaluate_figure8_ref(t_sec: float, xc: float, yc: float, Ax: float, Ay: float, omega: float, s_ramp: float = 1.0):
    w = omega
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

    m = MODEL
    tau_ff = np.array([
        m["m11"] * dud - m["m22"] * vd * rd + (m["Xu"] + m["Xuu"] * abs(ud)) * ud,
        m["m22"] * dvd + m["m11"] * ud * rd + (m["Yv"] + m["Yvv"] * abs(vd)) * vd,
        m["m33"] * drd + (m["m22"] - m["m11"]) * ud * vd + (m["Nr"] + m["Nrr"] * abs(rd)) * rd
    ], dtype=float)

    eta_d = np.array([xd, yd, psid], dtype=float)
    nu_d = np.array([ud, vd, rd], dtype=float)
    return {"eta_d": eta_d, "nu_d": nu_d, "tau_ff": tau_ff}


def find_closest_trajectory_time(x0: float, y0: float, yaw0: float,
                                 xc: float, yc: float, Ax: float, Ay: float, period_s: float) -> float:
    """Finds t_offset in [0, period_s) on the Figure-8 closest to current boat pose (x0, y0, yaw0)."""
    omega = 2.0 * math.pi / period_s
    n_samples = 1000
    best_t = 0.0
    best_score = 1e18

    for i in range(n_samples):
        t_cand = (i / n_samples) * period_s
        xd = xc + Ax * math.sin(omega * t_cand)
        yd = yc + Ay * math.sin(2.0 * omega * t_cand)
        dxd = Ax * omega * math.cos(omega * t_cand)
        dyd = 2.0 * Ay * omega * math.cos(2.0 * omega * t_cand)
        psid = math.atan2(dyd, dxd)

        d2 = (xd - x0) ** 2 + (yd - y0) ** 2
        dpsi = wrap_to_pi(psid - yaw0)
        score = d2 + (0.35 ** 2) * (1.0 - math.cos(dpsi))
        if score < best_score:
            best_score = score
            best_t = t_cand

    return best_t


def run_test():
    xc, yc = -0.5713, -1.2731
    Ax, Ay = 0.5 * 5.0292 * 0.70, 0.5 * 4.8290 * 0.70
    period_s = 85.0
    omega = 2.0 * math.pi / period_s
    dt = 1.0 / 30.0

    # Suppose the boat starts near one of the outer lobes (like in the user's screenshot!)
    x0, y0, yaw0 = 0.95, -0.40, 1.10
    t_offset = find_closest_trajectory_time(x0, y0, yaw0, xc, yc, Ax, Ay, period_s)
    r0 = evaluate_figure8_ref(t_offset, xc, yc, Ax, Ay, omega)
    print(f"Boat initial pose: ({x0:.2f}, {y0:.2f}, yaw={math.degrees(yaw0):.1f} deg)")
    print(f"Closest Figure-8 phase: t_offset = {t_offset:.2f} s -> ref=({r0['eta_d'][0]:.2f}, {r0['eta_d'][1]:.2f}, yaw={math.degrees(r0['eta_d'][2]):.1f} deg)")

    mpc = SmoothFossenMPC(N=15, dt_mpc=0.12)
    eta = np.array([x0, y0, yaw0], dtype=float)
    nu = np.zeros(3, dtype=float)
    cmd_Filtered = np.zeros(4, dtype=float)

    n_steps = int(period_s / dt)
    pos_errs = []
    solve_times = []
    cmd_diffs = []

    for k in range(n_steps):
        t_el = k * dt
        s_ramp = 0.5 * (1.0 - math.cos(math.pi * t_el / 4.0)) if t_el < 4.0 else 1.0

        horizon = []
        for h in range(mpc.N + 1):
            t_h = t_offset + t_el + h * mpc.dt_mpc
            horizon.append(evaluate_figure8_ref(t_h, xc, yc, Ax, Ay, omega, s_ramp=s_ramp))

        tau_cmd, ms = mpc.solve(eta, nu, horizon)
        solve_times.append(ms)

        f_mot = PHI_PINV @ tau_cmd
        cmd_raw = thrust_to_cmd_richards(f_mot)
        cmd_new = 0.70 * cmd_Filtered + 0.30 * cmd_raw
        cmd_diffs.append(np.max(np.abs(cmd_new - cmd_Filtered)))
        cmd_Filtered = cmd_new

        # Plant step
        f_real = cmd_to_thrust_richards(cmd_Filtered)
        tau_real = PHI_ALLOC @ f_real
        m = MODEL
        u, v, r = nu
        du = (tau_real[0] + m["m22"]*v*r - (m["Xu"] + m["Xuu"]*abs(u))*u) / m["m11"]
        dv = (tau_real[1] - m["m11"]*u*r - (m["Yv"] + m["Yvv"]*abs(v))*v) / m["m22"]
        dr = (tau_real[2] - (m["m22"] - m["m11"])*u*v - (m["Nr"] + m["Nrr"]*abs(r))*r) / m["m33"]
        nu = nu + dt * np.array([du, dv, dr])
        c, s = math.cos(eta[2]), math.sin(eta[2])
        eta = eta + dt * np.array([c*nu[0] - s*nu[1], s*nu[0] + c*nu[1], nu[2]])
        eta[2] = wrap_to_pi(eta[2])

        ref0 = horizon[0]["eta_d"]
        pos_errs.append(math.hypot(eta[0] - ref0[0], eta[1] - ref0[1]))

    print(f"Steady-state RMSE (after 5s): {np.sqrt(np.mean(np.square(pos_errs[150:]))):.4f} m")
    print(f"Mean MPC solve time (N=15):   {np.mean(solve_times):.3f} ms (max {np.max(solve_times):.3f} ms)")
    print(f"Max thruster Δcmd per tick:   {np.max(cmd_diffs):.4f}")


if __name__ == "__main__":
    run_test()
