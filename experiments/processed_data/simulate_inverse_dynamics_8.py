import csv
import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from matplotlib.patches import Rectangle

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
})

COLOR_REAL = '#000000'
COLOR_DESIRED = '#B22222'
COLOR_NAVY = '#003366'
COLOR_GREEN = '#2E8B57'
COLOR_GRAY = '#6B7280'

DEFAULT_MODEL = {
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

D_ARM = 0.65 / np.sqrt(2.0)
COS45 = np.cos(np.deg2rad(45.0))
SIN45 = np.sin(np.deg2rad(45.0))

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


def get_pool_bounds(data_dir):
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


def generate_figure8_reference(t, pool, period_s=55.0):
    xc = pool["x_center"]
    yc = pool["y_center"]
    Ax = pool["ax_70"]
    Ay = pool["ay_70"]
    w = 2.0 * np.pi / period_s

    xd = xc + Ax * np.sin(w * t)
    yd = yc + Ay * np.sin(2.0 * w * t)

    dxd = Ax * w * np.cos(w * t)
    dyd = 2.0 * Ay * w * np.cos(2.0 * w * t)

    ddxd = -Ax * (w ** 2) * np.sin(w * t)
    ddyd = -4.0 * Ay * (w ** 2) * np.sin(2.0 * w * t)

    dddxd = -Ax * (w ** 3) * np.cos(w * t)
    dddyd = -8.0 * Ay * (w ** 3) * np.cos(2.0 * w * t)

    psid = np.unwrap(np.arctan2(dyd, dxd))
    v2 = dxd ** 2 + dyd ** 2 + 1e-12
    dpsid = (dxd * ddyd - dyd * ddxd) / v2
    dv2_dt = 2.0 * (dxd * ddxd + dyd * ddyd)
    ddpsid = ((dxd * dddyd - dyd * dddxd) * v2 - (dxd * ddyd - dyd * ddxd) * dv2_dt) / (v2 ** 2)

    eta_d = np.column_stack([xd, yd, psid])
    deta_d = np.column_stack([dxd, dyd, dpsid])
    ddeta_d = np.column_stack([ddxd, ddyd, ddpsid])

    cos_p = np.cos(psid)
    sin_p = np.sin(psid)
    ud = cos_p * dxd + sin_p * dyd
    vd = -sin_p * dxd + cos_p * dyd
    rd = dpsid
    nu_d = np.column_stack([ud, vd, rd])

    return eta_d, deta_d, ddeta_d, nu_d


def cmd_to_thrust_richards(cmd):
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


def thrust_to_cmd_richards(T_des):
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


def wrap_to_pi(angle):
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def rotation_matrix_3dof(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0]
    ], dtype=float)


def fossen_coriolis_damping(nu, model):
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


def inverse_dynamics_control(eta, nu, eta_d, deta_d, ddeta_d, model, Kp, Kd):
    psi = float(eta[2])
    u, v, r = float(nu[0]), float(nu[1]), float(nu[2])

    J = rotation_matrix_3dof(psi)
    deta = J @ nu

    e_eta = np.array([
        eta_d[0] - eta[0],
        eta_d[1] - eta[1],
        wrap_to_pi(eta_d[2] - eta[2])
    ], dtype=float)
    de_eta = deta_d - deta

    a_eta = ddeta_d + Kd @ de_eta + Kp @ e_eta

    s_nu = np.array([-v * r, u * r, 0.0], dtype=float)
    a_nu = J.T @ a_eta - s_nu

    M_diag = np.array([model["m11"], model["m22"], model["m33"]], dtype=float)
    C_nu, D_nu = fossen_coriolis_damping(nu, model)

    tau_cmd = M_diag * a_nu + C_nu + D_nu

    f_motors = PHI_PINV @ tau_cmd
    cmd_norm = thrust_to_cmd_richards(f_motors)
    f_motors_sat = cmd_to_thrust_richards(cmd_norm)
    tau_applied = PHI_ALLOC @ f_motors_sat
    pwm_us = 1500.0 + 400.0 * cmd_norm

    return tau_applied, tau_cmd, pwm_us


def rk4_step_fossen(eta, nu, tau, dt, model):
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


def run_simulation():
    data_dir = Path(__file__).resolve().parent
    json_path = data_dir / "identified_models_pladypos.json"

    if json_path.exists():
        with open(json_path, "r", encoding="utf-8") as fp:
            models_data = json.load(fp)
        model = models_data.get("stage2_whitebox_refined_optimal_9param", DEFAULT_MODEL)
    else:
        model = DEFAULT_MODEL.copy()

    pool = get_pool_bounds(data_dir)

    fs = 30.0
    dt = 1.0 / fs
    period_s = 55.0
    t_final = period_s
    t = np.arange(0.0, t_final + 0.5 * dt, dt)
    N = len(t)

    eta_d, deta_d, ddeta_d, nu_d = generate_figure8_reference(t, pool, period_s=period_s)

    Kp = np.diag([2.25, 2.25, 4.84])
    Kd = np.diag([3.00, 3.00, 4.40])

    eta_sim = np.zeros((N, 3), dtype=float)
    nu_sim = np.zeros((N, 3), dtype=float)
    tau_sim = np.zeros((N, 3), dtype=float)
    pwm_sim = np.zeros((N, 4), dtype=float)

    eta_sim[0] = eta_d[0].copy()
    nu_sim[0] = nu_d[0].copy()

    for k in range(N - 1):
        tau_app, _, pwm_k = inverse_dynamics_control(
            eta_sim[k], nu_sim[k], eta_d[k], deta_d[k], ddeta_d[k], model, Kp, Kd
        )
        tau_sim[k] = tau_app
        pwm_sim[k] = pwm_k
        eta_sim[k + 1], nu_sim[k + 1] = rk4_step_fossen(eta_sim[k], nu_sim[k], tau_app, dt, model)

    tau_sim[-1], _, pwm_sim[-1] = inverse_dynamics_control(
        eta_sim[-1], nu_sim[-1], eta_d[-1], deta_d[-1], ddeta_d[-1], model, Kp, Kd
    )

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

    err_pos = np.hypot(eta_sim[:, 0] - eta_d[:, 0], eta_sim[:, 1] - eta_d[:, 1])
    rmse_pos = float(np.sqrt(np.mean(err_pos ** 2)))
    rmse_u = float(np.sqrt(np.mean((nu_sim[:, 0] - nu_d[:, 0]) ** 2)))
    rmse_v = float(np.sqrt(np.mean((nu_sim[:, 1] - nu_d[:, 1]) ** 2)))
    rmse_r = float(np.sqrt(np.mean((nu_sim[:, 2] - nu_d[:, 2]) ** 2)))

    print(f"Position RMSE: {rmse_pos:.6f} m")
    print(f"Velocity RMSE: u={rmse_u:.6f} m/s, v={rmse_v:.6f} m/s, r={rmse_r:.6f} rad/s")
    print(f"CSV saved: {out_csv}")

    fig, axes = plt.subplots(3, 2, figsize=(10.0, 8.4))

    ax = axes[0, 0]
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

    ax = axes[1, 0]
    ax.plot(t, nu_d[:, 0], color=COLOR_DESIRED, linestyle='-', linewidth=1.7, label='Desired', zorder=2)
    ax.plot(t, nu_sim[:, 0], color=COLOR_REAL, linestyle='--', linewidth=1.2, label='Inverse Dynamics', zorder=3)
    ax.set_xlabel(r'Time $t \ [\mathrm{s}]$')
    ax.set_ylabel(r'Surge Velocity $u \ [\mathrm{m/s}]$')
    ax.set_xlim(0, t[-1])
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right', frameon=True, edgecolor='black', fontsize=7.5)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

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

    ax = axes[2, 0]
    ax.plot(t, nu_d[:, 2], color=COLOR_DESIRED, linestyle='-', linewidth=1.7, label='Desired', zorder=2)
    ax.plot(t, nu_sim[:, 2], color=COLOR_REAL, linestyle='--', linewidth=1.2, label='Inverse Dynamics', zorder=3)
    ax.set_xlabel(r'Time $t \ [\mathrm{s}]$')
    ax.set_ylabel(r'Yaw Rate $r \ [\mathrm{rad/s}]$')
    ax.set_xlim(0, t[-1])
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend(loc='upper right', frameon=True, edgecolor='black', fontsize=7.5)
    ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

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
    plt.show()


if __name__ == "__main__":
    run_simulation()
