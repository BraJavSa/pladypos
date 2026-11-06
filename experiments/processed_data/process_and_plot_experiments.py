#!/usr/bin/env python3
"""
process_and_plot_experiments.py

Extrae y separa los datos de los experimentos (.mcap) en /home/brayan/ros2_ws/src/pladypos/experiments
SIN aplicar ningun filtrado offline adicional (toma directamente los datos ya filtrados en linea
durante el experimento en ROS 2):
1. De /usv5/odometry/filtered (30 Hz):
   - Posiciones planares: x, y [m]
   - Orientacion de guinada continua (unwrapped, sin saltos): yaw [rad], yaw_deg [deg]
2. Velocidades en el marco del cuerpo del USV (u, v, r):
   - De /usv5/visual_odom (velocidades lineales proyectadas al cuerpo con yaw):
       * u (Surge, avance frontal en cuerpo [m/s])
       * v (Sway, desplazamiento lateral en cuerpo [m/s])
   - De /usv5/imu/data:
       * r (Yaw rate, velocidad angular de guinada en cuerpo [rad/s])
3. De /usv5/pwm_out (acciones de control enviadas a cada uno de los 4 motores en configuracion X):
   - Motor 1 (FR - Front Right, Pin A0): pwm_us_m1_FR [1100..1900 us], pwm_norm_m1_FR [-1..1]
   - Motor 2 (FL - Front Left,  Pin A1): pwm_us_m2_FL [1100..1900 us], pwm_norm_m2_FL [-1..1]
   - Motor 3 (BR - Back Right,  Pin A2): pwm_us_m3_BR [1100..1900 us], pwm_norm_m3_BR [-1..1]
   - Motor 4 (BL - Back Left,   Pin A3): pwm_us_m4_BL [1100..1900 us], pwm_norm_m4_BL [-1..1]
"""

import csv
import math
import struct
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.gridspec import GridSpec
from matplotlib.ticker import FormatStrFormatter


plt.rcParams.update({
    "font.size": 9.0,
    "axes.labelsize": 9.0,
    "axes.titlesize": 9.5,
    "xtick.labelsize": 8.0,
    "ytick.labelsize": 8.0,
    "legend.fontsize": 7.8,
    "font.family": "serif",
    "mathtext.fontset": "cm",
    "figure.facecolor": "white",
})


# ============================================================================
# Lector directo de archivos ROS 2 MCAP (CDR Little-Endian)
# ============================================================================

def _read_str(buf: bytes, offset: int):
    length = struct.unpack_from("<I", buf, offset)[0]
    offset += 4
    s = buf[offset : offset + length].decode("utf-8", errors="replace").rstrip("\x00")
    return s, offset + length


def _align(offset: int, alignment: int) -> int:
    rem = offset % alignment
    return offset if rem == 0 else offset + (alignment - rem)


def parse_odometry_cdr(buf: bytes):
    """Deserializa nav_msgs/msg/Odometry en formato CDR."""
    off = 4
    sec, nanosec = struct.unpack_from("<iI", buf, off)
    off += 8
    frame_id, off = _read_str(buf, off)
    off = _align(off, 4)
    child_frame_id, off = _read_str(buf, off)
    off = 4 + ((off - 4 + 7) & ~7)
    px, py, pz, qx, qy, qz, qw = struct.unpack_from("<7d", buf, off)
    off += 56
    off += 36 * 8
    vx, vy, vz, wx, wy, wz = struct.unpack_from("<6d", buf, off)
    stamp = float(sec) + float(nanosec) * 1e-9
    return stamp, px, py, pz, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz


def parse_imu_cdr(buf: bytes):
    """Deserializa sensor_msgs/msg/Imu en formato CDR."""
    off = 4
    sec, nanosec = struct.unpack_from("<iI", buf, off)
    off += 8
    frame_id, off = _read_str(buf, off)
    off = 4 + ((off - 4 + 7) & ~7)
    qx, qy, qz, qw = struct.unpack_from("<4d", buf, off)
    off += 32
    off += 9 * 8
    wx, wy, wz = struct.unpack_from("<3d", buf, off)
    stamp = float(sec) + float(nanosec) * 1e-9
    return stamp, qx, qy, qz, qw, wx, wy, wz


def parse_float32_multiarray_cdr(buf: bytes):
    """Deserializa std_msgs/msg/Float32MultiArray en formato CDR."""
    off = 4
    num_dims = struct.unpack_from("<I", buf, off)[0]
    off += 4
    for _ in range(num_dims):
        _, off = _read_str(buf, off)
        off = _align(off, 4)
        off += 8
    off = _align(off, 4)
    _data_offset = struct.unpack_from("<I", buf, off)[0]
    off += 4
    seq_len = struct.unpack_from("<I", buf, off)[0]
    off += 4
    vals = struct.unpack_from(f"<{seq_len}f", buf, off)
    return vals


def read_experiment_mcap(mcap_path: Path):
    """Extrae los topicos grabados en un archivo .mcap."""
    with open(mcap_path, "rb") as f:
        data = f.read()

    channels = {}
    odom_filtered = []
    visual_odom = []
    imu_data = []
    pwm_out = []
    motor_fb = []

    def _parse_block(buf: bytes, start: int, end: int):
        p = start
        while p < end:
            op = buf[p]
            rlen = struct.unpack_from("<Q", buf, p + 1)[0]
            rs = p + 9
            if op == 0x04:  # Channel
                cid, _sid = struct.unpack_from("<HH", buf, rs)
                topic, _ = _read_str(buf, rs + 4)
                channels[cid] = topic
            elif op == 0x05:  # Message
                cid = struct.unpack_from("<H", buf, rs)[0]
                log_time = struct.unpack_from("<Q", buf, rs + 6)[0] * 1e-9
                mdata = buf[rs + 22 : rs + rlen]
                topic = channels.get(cid)
                if topic == "/usv5/odometry/filtered":
                    odom_filtered.append((log_time,) + parse_odometry_cdr(mdata))
                elif topic == "/usv5/visual_odom":
                    visual_odom.append((log_time,) + parse_odometry_cdr(mdata))
                elif topic in ("/usv5/imu/data", "/usv5/imu/data_raw"):
                    imu_data.append((log_time,) + parse_imu_cdr(mdata))
                elif topic == "/usv5/pwm_out":
                    vals = parse_float32_multiarray_cdr(mdata)
                    if len(vals) >= 4:
                        pwm_out.append((log_time, vals[0], vals[1], vals[2], vals[3]))
                elif topic == "/usv5/motor_feedback":
                    vals = parse_float32_multiarray_cdr(mdata)
                    if len(vals) >= 4:
                        motor_fb.append((log_time, vals[0], vals[1], vals[2], vals[3]))
            elif op == 0x06:  # Chunk
                off = rs + 28
                comp, off = _read_str(buf, off)
                clen = struct.unpack_from("<Q", buf, off)[0]
                off += 8
                if comp == "":
                    _parse_block(buf, off, off + clen)
            p = rs + rlen

    _parse_block(data, 8, len(data) - 8)

    return {
        "odom_filtered": np.array(odom_filtered, dtype=np.float64),
        "visual_odom": np.array(visual_odom, dtype=np.float64),
        "imu_data": np.array(imu_data, dtype=np.float64),
        "pwm_out": np.array(pwm_out, dtype=np.float64),
        "motor_fb": np.array(motor_fb, dtype=np.float64),
    }


# ============================================================================
# Extraccion y Separacion Directa de Datos (Sin Filtrado Offline Adicional)
# ============================================================================

def quat_to_euler_array(qx, qy, qz, qw):
    """Convierte arreglos de cuaterniones a (roll, pitch, yaw) en radianes."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def process_experiment(exp_dir: Path):
    """
    Separa directamente los datos del experimento sobre los instantes de muestreo
    del topico /usv5/odometry/filtered (~30 Hz), sin aplicar filtros post-proceso.
    """
    mcap_files = sorted(exp_dir.glob("*.mcap"))
    if not mcap_files:
        raise FileNotFoundError(f"No se encontro archivo .mcap en {exp_dir}")

    raw = read_experiment_mcap(mcap_files[0])
    odom = raw["odom_filtered"]
    if odom.ndim != 2 or len(odom) == 0:
        raise RuntimeError(f"Sin mensajes de /usv5/odometry/filtered en {exp_dir}")

    # Eliminar duplicados exactos de estampa de tiempo (producidos por el bridge DDS/Zenoh)
    _, unique_idx = np.unique(odom[:, 1], return_index=True)
    odom = odom[np.sort(unique_idx)]

    t_log = odom[:, 0]
    stamp = odom[:, 1]
    t_rel = t_log - t_log[0]

    # 1. Posiciones X, Y [m] directamente de /usv5/odometry/filtered
    x = odom[:, 2]
    y = odom[:, 3]

    # 2. Orientacion Yaw continua (sin saltos en +-180 deg / +-pi rad)
    qx, qy, qz, qw = odom[:, 5], odom[:, 6], odom[:, 7], odom[:, 8]
    _roll, _pitch, yaw_wrapped = quat_to_euler_array(qx, qy, qz, qw)
    yaw_cont = np.unwrap(yaw_wrapped)
    yaw_deg_cont = np.degrees(yaw_cont)

    # 3. Las 3 velocidades en el marco del cuerpo del USV (u, v, r) tomadas de los topicos grabados:
    #    - Velocidades lineales (vx_cam, vy_cam) publicadas en /usv5/visual_odom:
    #      En el marco odom: vx_odom = vx_cam, vy_odom = -vy_cam
    #      Rotacion al cuerpo del barco con yaw:
    #        u =  vx_odom * cos(yaw) + vy_odom * sin(yaw)
    #        v = -vx_odom * sin(yaw) + vy_odom * cos(yaw)
    #    - Velocidad angular r = angular_velocity.z publicada en /usv5/imu/data
    vis = raw["visual_odom"]
    if len(vis) > 0:
        _, idx_v = np.unique(vis[:, 1], return_index=True)
        vis = vis[np.sort(idx_v)]
        vx_odom = np.interp(t_log, vis[:, 0], vis[:, 9])
        vy_odom = np.interp(t_log, vis[:, 0], -vis[:, 10])
    else:
        vx_odom = odom[:, 9]
        vy_odom = odom[:, 10]

    imu = raw["imu_data"]
    if len(imu) > 0:
        imu = imu[np.argsort(imu[:, 0])]
        r_body = np.interp(t_log, imu[:, 0], imu[:, 8])
        before_imu = t_log < (imu[0, 0] - 0.2)
        if np.any(before_imu):
            r_odom_fallback = np.gradient(yaw_cont, stamp - stamp[0])
            r_body[before_imu] = r_odom_fallback[before_imu]
    else:
        r_body = odom[:, 14]

    cos_y = np.cos(yaw_cont)
    sin_y = np.sin(yaw_cont)
    u_body = vx_odom * cos_y + vy_odom * sin_y
    v_body = -vx_odom * sin_y + vy_odom * cos_y

    # 4. Acciones de control: PWM de cada uno de los 4 motores (/usv5/pwm_out)
    #    - Motor 1 (idx 0): FR (Front Right, Pin A0)
    #    - Motor 2 (idx 1): FL (Front Left,  Pin A1)
    #    - Motor 3 (idx 2): BR (Back Right,  Pin A2)
    #    - Motor 4 (idx 3): BL (Back Left,   Pin A3)
    pwm_src = raw["pwm_out"] if len(raw["pwm_out"]) > 0 else raw["motor_fb"]
    if len(pwm_src) > 0:
        t_pwm = pwm_src[:, 0]
        idx_prev = np.searchsorted(t_pwm, t_log, side="right") - 1
        idx_prev = np.clip(idx_prev, 0, len(t_pwm) - 1)
        u_m1_FR = np.clip(pwm_src[idx_prev, 1], -1.0, 1.0)
        u_m2_FL = np.clip(pwm_src[idx_prev, 2], -1.0, 1.0)
        u_m3_BR = np.clip(pwm_src[idx_prev, 3], -1.0, 1.0)
        u_m4_BL = np.clip(pwm_src[idx_prev, 4], -1.0, 1.0)
    else:
        u_m1_FR = np.zeros_like(t_rel)
        u_m2_FL = np.zeros_like(t_rel)
        u_m3_BR = np.zeros_like(t_rel)
        u_m4_BL = np.zeros_like(t_rel)

    pwm_us_m1_FR = np.round(1500.0 + 400.0 * u_m1_FR, 1)
    pwm_us_m2_FL = np.round(1500.0 + 400.0 * u_m2_FL, 1)
    pwm_us_m3_BR = np.round(1500.0 + 400.0 * u_m3_BR, 1)
    pwm_us_m4_BL = np.round(1500.0 + 400.0 * u_m4_BL, 1)

    return {
        "exp_name": exp_dir.name,
        "t": t_rel,
        "timestamp": stamp,
        "x": x,
        "y": y,
        "yaw": yaw_cont,
        "yaw_deg": yaw_deg_cont,
        "u": u_body,
        "v": v_body,
        "r": r_body,
        "pwm_us_m1_FR": pwm_us_m1_FR,
        "pwm_us_m2_FL": pwm_us_m2_FL,
        "pwm_us_m3_BR": pwm_us_m3_BR,
        "pwm_us_m4_BL": pwm_us_m4_BL,
        "pwm_norm_m1_FR": u_m1_FR,
        "pwm_norm_m2_FL": u_m2_FL,
        "pwm_norm_m3_BR": u_m3_BR,
        "pwm_norm_m4_BL": u_m4_BL,
    }


# ============================================================================
# Exportacion a CSV (Solo posiciones, yaw continuo, velocidades cuerpo y PWM)
# ============================================================================

CSV_COLUMNS = [
    "t",
    "timestamp",
    "x",
    "y",
    "yaw",
    "yaw_deg",
    "u",
    "v",
    "r",
    "pwm_us_m1_FR",
    "pwm_us_m2_FL",
    "pwm_us_m3_BR",
    "pwm_us_m4_BL",
    "pwm_norm_m1_FR",
    "pwm_norm_m2_FL",
    "pwm_norm_m3_BR",
    "pwm_norm_m4_BL",
]


def save_experiment_csv(exp_data: dict, out_csv: Path):
    """Guarda los datos separados de un experimento en un archivo CSV."""
    n = len(exp_data["t"])
    with open(out_csv, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(CSV_COLUMNS)
        for i in range(n):
            writer.writerow([
                f"{exp_data['t'][i]:.4f}",
                f"{exp_data['timestamp'][i]:.6f}",
                f"{exp_data['x'][i]:.5f}",
                f"{exp_data['y'][i]:.5f}",
                f"{exp_data['yaw'][i]:.5f}",
                f"{exp_data['yaw_deg'][i]:.3f}",
                f"{exp_data['u'][i]:.5f}",
                f"{exp_data['v'][i]:.5f}",
                f"{exp_data['r'][i]:.5f}",
                f"{exp_data['pwm_us_m1_FR'][i]:.1f}",
                f"{exp_data['pwm_us_m2_FL'][i]:.1f}",
                f"{exp_data['pwm_us_m3_BR'][i]:.1f}",
                f"{exp_data['pwm_us_m4_BL'][i]:.1f}",
                f"{exp_data['pwm_norm_m1_FR'][i]:.4f}",
                f"{exp_data['pwm_norm_m2_FL'][i]:.4f}",
                f"{exp_data['pwm_norm_m3_BR'][i]:.4f}",
                f"{exp_data['pwm_norm_m4_BL'][i]:.4f}",
            ])


def save_combined_csv(all_exp_data: list, out_csv: Path):
    """Guarda todos los experimentos en un unico CSV consolidado."""
    with open(out_csv, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["experiment"] + CSV_COLUMNS)
        for exp_data in all_exp_data:
            name = exp_data["exp_name"]
            n = len(exp_data["t"])
            for i in range(n):
                writer.writerow([
                    name,
                    f"{exp_data['t'][i]:.4f}",
                    f"{exp_data['timestamp'][i]:.6f}",
                    f"{exp_data['x'][i]:.5f}",
                    f"{exp_data['y'][i]:.5f}",
                    f"{exp_data['yaw'][i]:.5f}",
                    f"{exp_data['yaw_deg'][i]:.3f}",
                    f"{exp_data['u'][i]:.5f}",
                    f"{exp_data['v'][i]:.5f}",
                    f"{exp_data['r'][i]:.5f}",
                    f"{exp_data['pwm_us_m1_FR'][i]:.1f}",
                    f"{exp_data['pwm_us_m2_FL'][i]:.1f}",
                    f"{exp_data['pwm_us_m3_BR'][i]:.1f}",
                    f"{exp_data['pwm_us_m4_BL'][i]:.1f}",
                    f"{exp_data['pwm_norm_m1_FR'][i]:.4f}",
                    f"{exp_data['pwm_norm_m2_FL'][i]:.4f}",
                    f"{exp_data['pwm_norm_m3_BR'][i]:.4f}",
                    f"{exp_data['pwm_norm_m4_BL'][i]:.4f}",
                ])


# ============================================================================
# Graficacion de Pagina por Experimento en PDF (Subplots Individuales)
# ============================================================================

def _despike_signal(sig: np.ndarray, window: int = 5, threshold: float = 0.22) -> np.ndarray:
    out = np.asarray(sig, dtype=float).copy()
    n = len(out)
    half = window // 2
    for i in range(half, n - half):
        med = np.median(out[i - half:i + half + 1])
        if abs(out[i] - med) > threshold:
            out[i] = med
    return out


def _plot_single_motor(ax, t, pwm_us, color, ylabel_str, label_str, duration):
    ax.plot(t, pwm_us, color=color, linewidth=1.3, label=label_str)
    ax.axhline(1500.0, color="black", linestyle=":", linewidth=0.9, label=r"Neutral ($1500 \ \mu\mathrm{s}$)")
    ax.set_xlabel(r"Time $t \ [\mathrm{s}]$")
    ax.set_ylabel(ylabel_str)
    ax.set_xlim(0, duration)
    ax.set_ylim(1050.0, 1950.0)
    ax.set_yticks([1100, 1300, 1500, 1700, 1900])
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="upper right", frameon=True, edgecolor="black", fontsize=7.5)

    ax_norm = ax.twinx()
    ax_norm.set_ylim(-1.125, 1.125)
    ax_norm.set_yticks([-1.0, -0.5, 0.0, 0.5, 1.0])
    ax_norm.set_ylabel(r"Command $[-1, 1]$", color="#4B5563", fontsize=8.0)


def plot_experiment_page(exp_data: dict, page_idx: int, total_pages: int):
    """
    Academic IEEE-style 4x2 page per experiment (English, no titles, no numbering,
    only the 3 velocities u, v, r and the motor PWM control actions).
    """
    t = exp_data["t"]
    u = _despike_signal(exp_data["u"], window=5, threshold=0.22)
    v = _despike_signal(exp_data["v"], window=5, threshold=0.22)
    r = _despike_signal(exp_data["r"], window=5, threshold=0.60)

    m1_us = exp_data["pwm_us_m1_FR"]
    m2_us = exp_data["pwm_us_m2_FL"]
    m3_us = exp_data["pwm_us_m3_BR"]
    m4_us = exp_data["pwm_us_m4_BL"]

    duration = t[-1] if len(t) > 0 else 0.0

    fig = plt.figure(figsize=(10.0, 11.0), dpi=300)
    gs = GridSpec(4, 2, figure=fig, height_ratios=[1.0, 1.0, 1.0, 1.0], hspace=0.34, wspace=0.32)

    # Row 1, Left: Body Surge Velocity u(t)
    ax_u = fig.add_subplot(gs[0, 0])
    ax_u.plot(t, u, color="#000000", linewidth=1.25, label=r"$u(t)$")
    ax_u.axhline(0.0, color="#6B7280", linestyle=":", linewidth=0.8)
    ax_u.set_xlabel(r"Time $t \ [\mathrm{s}]$")
    ax_u.set_ylabel(r"Surge Velocity $u \ [\mathrm{m/s}]$")
    ax_u.set_xlim(0, duration)
    ax_u.grid(True, linestyle="--", alpha=0.6)
    ax_u.legend(loc="upper right", frameon=True, edgecolor="black", fontsize=7.5)
    ax_u.yaxis.set_major_formatter(FormatStrFormatter("%.2f"))

    # Row 1, Right: Body Sway Velocity v(t)
    ax_v = fig.add_subplot(gs[0, 1])
    ax_v.plot(t, v, color="#000000", linewidth=1.25, label=r"$v(t)$")
    ax_v.axhline(0.0, color="#6B7280", linestyle=":", linewidth=0.8)
    ax_v.set_xlabel(r"Time $t \ [\mathrm{s}]$")
    ax_v.set_ylabel(r"Sway Velocity $v \ [\mathrm{m/s}]$")
    ax_v.set_xlim(0, duration)
    ax_v.grid(True, linestyle="--", alpha=0.6)
    ax_v.legend(loc="upper right", frameon=True, edgecolor="black", fontsize=7.5)
    ax_v.yaxis.set_major_formatter(FormatStrFormatter("%.2f"))

    # Row 2, Left: Body Yaw Rate r(t)
    ax_r = fig.add_subplot(gs[1, 0])
    ax_r.plot(t, r, color="#000000", linewidth=1.25, label=r"$r(t)$")
    ax_r.axhline(0.0, color="#6B7280", linestyle=":", linewidth=0.8)
    ax_r.set_xlabel(r"Time $t \ [\mathrm{s}]$")
    ax_r.set_ylabel(r"Yaw Rate $r \ [\mathrm{rad/s}]$")
    ax_r.set_xlim(0, duration)
    ax_r.grid(True, linestyle="--", alpha=0.6)
    ax_r.legend(loc="upper right", frameon=True, edgecolor="black", fontsize=7.5)
    ax_r.yaxis.set_major_formatter(FormatStrFormatter("%.2f"))

    # Row 2, Right: All 4 Thrusters PWM Overview
    ax_all = fig.add_subplot(gs[1, 1])
    ax_all.plot(t, m1_us, color="#003366", linestyle="-", linewidth=1.1, label="M1 (FR)")
    ax_all.plot(t, m2_us, color="#2E8B57", linestyle="--", linewidth=1.1, label="M2 (FL)")
    ax_all.plot(t, m3_us, color="#D97706", linestyle="-.", linewidth=1.1, label="M3 (BR)")
    ax_all.plot(t, m4_us, color="#B22222", linestyle=":", linewidth=1.2, label="M4 (BL)")
    ax_all.axhline(1500.0, color="black", linestyle=":", linewidth=0.8)
    ax_all.set_xlabel(r"Time $t \ [\mathrm{s}]$")
    ax_all.set_ylabel(r"Thruster PWM $[\mu\mathrm{s}]$")
    ax_all.set_xlim(0, duration)
    ax_all.set_ylim(1050.0, 1950.0)
    ax_all.set_yticks([1100, 1300, 1500, 1700, 1900])
    ax_all.grid(True, linestyle="--", alpha=0.6)
    ax_all.legend(loc="upper right", ncol=2, frameon=True, edgecolor="black", fontsize=7.0)

    # Row 3, Left: Motor 1 — FR (Front Right)
    ax_m1 = fig.add_subplot(gs[2, 0])
    _plot_single_motor(ax_m1, t, m1_us, "#003366", r"M1 (FR) PWM $[\mu\mathrm{s}]$", "M1 (FR)", duration)

    # Row 3, Right: Motor 2 — FL (Front Left)
    ax_m2 = fig.add_subplot(gs[2, 1])
    _plot_single_motor(ax_m2, t, m2_us, "#2E8B57", r"M2 (FL) PWM $[\mu\mathrm{s}]$", "M2 (FL)", duration)

    # Row 4, Left: Motor 3 — BR (Back Right)
    ax_m3 = fig.add_subplot(gs[3, 0])
    _plot_single_motor(ax_m3, t, m3_us, "#D97706", r"M3 (BR) PWM $[\mu\mathrm{s}]$", "M3 (BR)", duration)

    # Row 4, Right: Motor 4 — BL (Back Left)
    ax_m4 = fig.add_subplot(gs[3, 1])
    _plot_single_motor(ax_m4, t, m4_us, "#B22222", r"M4 (BL) PWM $[\mu\mathrm{s}]$", "M4 (BL)", duration)

    fig.subplots_adjust(top=0.97, bottom=0.05, left=0.08, right=0.91)
    return fig


# ============================================================================
# Punto de Entrada Principal
# ============================================================================

def main():
    out_dir = Path(__file__).resolve().parent
    experiments_root = out_dir.parent

    exp_dirs = sorted([
        d for d in experiments_root.glob("exp_*")
        if d.is_dir() and list(d.glob("*.mcap"))
    ])

    if not exp_dirs:
        print(f"[Error] No se encontraron carpetas exp_* con archivos .mcap en {experiments_root}")
        return

    print(f"Extrayendo datos de {len(exp_dirs)} experimentos desde {experiments_root}...")
    all_exp_data = []

    for exp_dir in exp_dirs:
        exp_data = process_experiment(exp_dir)
        all_exp_data.append(exp_data)

        csv_path = out_dir / f"{exp_data['exp_name']}.csv"
        save_experiment_csv(exp_data, csv_path)
        print(
            f"  [OK] {exp_data['exp_name']}: {len(exp_data['t'])} muestras "
            f"({exp_data['t'][-1]:.2f} s) -> {csv_path.name}"
        )

    combined_csv_path = out_dir / "all_experiments.csv"
    save_combined_csv(all_exp_data, combined_csv_path)
    print(f"  [OK] CSV consolidado guardado en: {combined_csv_path.name}")

    pdf_path = out_dir / "experiments_report.pdf"
    with PdfPages(pdf_path) as pdf:
        total = len(all_exp_data)
        for i, exp_data in enumerate(all_exp_data, start=1):
            fig = plot_experiment_page(exp_data, i, total)
            pdf.savefig(fig, bbox_inches="tight")
            plt.close(fig)
            print(f"  [PDF] Pagina {i}/{total} generada para {exp_data['exp_name']}")

    print(f"\n[Completado] Reporte PDF multipagina generado en:\n  {pdf_path}")


if __name__ == "__main__":
    main()
