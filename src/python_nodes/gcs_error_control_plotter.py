#!/usr/bin/env python3
"""
gcs_error_control_plotter.py
============================
Real-Time GCS Live Plotter (runs on the Laptop in ROS_DOMAIN_ID=0).

Reads:
  1. /usv5/odometry/filtered (nav_msgs/Odometry): current boat pose (x, y, psi) and velocities (u, v, r)
  2. /pool_boundary (visualization_msgs/Marker, ns="figure8_target"): desired position (xd, yd)
     (and computes desired tangent heading psi_d from the Figure-8 geometry)
  3. /usv5/pwm_out (std_msgs/Float32MultiArray): 4 thruster PWM commands from the boat controller,
     reconstructing the exact applied control actions (Tu [N], Tv [N], Tr [N*m]) via the Richards
     thrust model and X-thruster allocation matrix PHI_ALLOC.

Displays a real-time window with 3 subplots (dual Y-axis on each):
  - Subplot 1: Longitudinal Error e_u [m]  & Control Action T_u [N]
  - Subplot 2: Lateral Error      e_v [m]  & Control Action T_v [N]
  - Subplot 3: Heading Error      e_psi [rad] & Control Action T_r [N*m]

Also saves a PNG/PDF summary figure when closed.
"""

import math
import os
import threading
import time
import yaml
from collections import deque
from datetime import datetime
from pathlib import Path
import numpy as np

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker


# ==============================================================================
# THRUSTER ALLOCATION & RICHARDS FORWARD MODEL (cmd in [-1, 1] -> Thrust [N])
# ==============================================================================
D_ARM = 0.65 / math.sqrt(2.0)  # 0.459619 m
COS45 = math.cos(math.radians(45.0))
SIN45 = math.sin(math.radians(45.0))

# Columns: [Front-Right (FR), Front-Left (FL), Back-Right (BR), Back-Left (BL)]
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


def cmd_to_thrust_richards(cmd: np.ndarray) -> np.ndarray:
    """Maps normalized thruster commands [-1, 1] to physical thrust [N]."""
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


def wrap_to_pi(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def quat_to_yaw(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class GCSErrorControlPlotter(Node):
    def __init__(self):
        super().__init__('gcs_error_control_plotter')

        self.declare_parameter('center_x', -0.5713)
        self.declare_parameter('center_y', -1.2731)
        self.declare_parameter('span_x', 5.0292)
        self.declare_parameter('span_y', 4.8290)
        self.declare_parameter('scale_factor', 0.70)
        self.declare_parameter('window_sec', 60.0)  # Rolling window in seconds (or full history)

        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.span_x = float(self.get_parameter('span_x').value)
        self.span_y = float(self.get_parameter('span_y').value)
        self.scale_factor = float(self.get_parameter('scale_factor').value)
        self.ax_8 = 0.5 * self.span_x * self.scale_factor
        self.ay_8 = 0.5 * self.span_y * self.scale_factor
        self.window_sec = float(self.get_parameter('window_sec').value)

        self._load_motor_mapping()

        # Precompute lookup table along the Figure-8 to recover tangent heading psi_d from (xd, yd)
        self.n_lut = 720
        thetas = np.linspace(0.0, 2.0 * math.pi, self.n_lut, endpoint=False)
        self.lut_x = self.center_x + self.ax_8 * np.sin(thetas)
        self.lut_y = self.center_y + self.ay_8 * np.sin(2.0 * thetas)
        dx_lut = self.ax_8 * np.cos(thetas)
        dy_lut = 2.0 * self.ay_8 * np.cos(2.0 * thetas)
        self.lut_psi = np.arctan2(dy_lut, dx_lut)
        self.prev_xd = None
        self.prev_yd = None
        self.prev_ref_time = None

        # State lock & variables
        self.lock = threading.Lock()
        self.has_odom = False
        self.has_ref = False
        self.start_time = None

        self.x = self.center_x
        self.y = self.center_y
        self.yaw = 0.0

        self.xd = self.center_x
        self.yd = self.center_y
        self.psid = 0.0

        self.Tu = 0.0
        self.Tv = 0.0
        self.Tr = 0.0

        # Time-series buffers (keep up to 10 minutes at 20 Hz = 12000 samples)
        max_len = 12000
        self.t_buf = deque(maxlen=max_len)
        self.eu_buf = deque(maxlen=max_len)
        self.ev_buf = deque(maxlen=max_len)
        self.epsi_buf = deque(maxlen=max_len)
        self.Tu_buf = deque(maxlen=max_len)
        self.Tv_buf = deque(maxlen=max_len)
        self.Tr_buf = deque(maxlen=max_len)

        # ROS 2 Subscriptions on GCS (ROS_DOMAIN_ID=0)
        self.create_subscription(Odometry, '/usv5/odometry/filtered', self.odom_cb, 10)
        self.create_subscription(Marker, '/pool_boundary', self.marker_cb, 10)
        self.create_subscription(Float32MultiArray, '/usv5/pwm_out', self.pwm_cb, 10)

        # 20 Hz sampling timer to record synchronized series whenever reference & odom are active
        self.create_timer(0.05, self.sample_tick)

        self.get_logger().info(
            "GCS Error & Control Plotter initialized!\n"
            "  Listening to: /usv5/odometry/filtered, /pool_boundary (figure8_target), /usv5/pwm_out\n"
            "  Plotting: [1] (e_u, T_u), [2] (e_v, T_v), [3] (e_psi, T_r)"
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

    def odom_cb(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        with self.lock:
            self.x = float(pos.x)
            self.y = float(pos.y)
            self.yaw = quat_to_yaw(ori.w, ori.x, ori.y, ori.z)
            self.has_odom = True

    def marker_cb(self, msg: Marker):
        if msg.ns != "figure8_target" or msg.action != Marker.ADD:
            return
        now_s = time.monotonic()
        xd_new = float(msg.pose.position.x)
        yd_new = float(msg.pose.position.y)

        # Estimate desired heading psi_d from motion direction of target sphere + Figure-8 LUT
        with self.lock:
            vel_angle = None
            if self.prev_xd is not None and self.prev_ref_time is not None:
                dt = now_s - self.prev_ref_time
                dx = xd_new - self.prev_xd
                dy = yd_new - self.prev_yd
                if dt > 1e-3 and math.hypot(dx, dy) > 1e-3:
                    vel_angle = math.atan2(dy, dx)

            d2 = (self.lut_x - xd_new) ** 2 + (self.lut_y - yd_new) ** 2
            if vel_angle is not None:
                dpsi = np.arctan2(np.sin(self.lut_psi - vel_angle), np.cos(self.lut_psi - vel_angle))
                score = d2 + (0.25 ** 2) * (dpsi ** 2)
            else:
                dpsi = np.arctan2(np.sin(self.lut_psi - self.psid), np.cos(self.lut_psi - self.psid))
                score = d2 + (0.15 ** 2) * (dpsi ** 2)

            idx = int(np.argmin(score))
            self.xd = xd_new
            self.yd = yd_new
            self.psid = float(self.lut_psi[idx])
            self.prev_xd = xd_new
            self.prev_yd = yd_new
            self.prev_ref_time = now_s
            self.has_ref = True

    def pwm_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        u_FR = u_FL = u_BR = u_BL = 0.0
        for channel, position in self.motor_map.items():
            ch_idx = int(channel) - 1
            if 0 <= ch_idx < 4:
                val = float(msg.data[ch_idx])
                if "Front Right" in position or "FR" in position:
                    u_FR = val
                elif "Front Left" in position or "FL" in position:
                    u_FL = val
                elif "Back Right" in position or "BR" in position:
                    u_BR = val
                elif "Back Left" in position or "BL" in position:
                    u_BL = val

        f_motors = cmd_to_thrust_richards(np.array([u_FR, u_FL, u_BR, u_BL], dtype=float))
        tau = PHI_ALLOC @ f_motors
        with self.lock:
            self.Tu = float(tau[0])
            self.Tv = float(tau[1])
            self.Tr = float(tau[2])

    def sample_tick(self):
        with self.lock:
            if not self.has_odom or not self.has_ref:
                return
            now_s = time.monotonic()
            if self.start_time is None:
                self.start_time = now_s
            t_rel = now_s - self.start_time

            ex = self.xd - self.x
            ey = self.yd - self.y
            c_psi = math.cos(self.yaw)
            s_psi = math.sin(self.yaw)

            # Body-frame errors: e_u (longitudinal), e_v (lateral), e_psi (yaw)
            e_u = c_psi * ex + s_psi * ey
            e_v = -s_psi * ex + c_psi * ey
            e_psi = wrap_to_pi(self.psid - self.yaw)

            self.t_buf.append(t_rel)
            self.eu_buf.append(e_u)
            self.ev_buf.append(e_v)
            self.epsi_buf.append(e_psi)
            self.Tu_buf.append(self.Tu)
            self.Tv_buf.append(self.Tv)
            self.Tr_buf.append(self.Tr)


def main(args=None):
    os.environ["ROS_DOMAIN_ID"] = "0"
    rclpy.init(args=args)
    node = GCSErrorControlPlotter()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'default')
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 8.0), sharex=True)
    fig.canvas.manager.set_window_title("PlaDyPos USV - Tracking Errors & Control Actions")

    # Dual Y-axes for Control Actions
    ax1_r = ax1.twinx()
    ax2_r = ax2.twinx()
    ax3_r = ax3.twinx()

    # Subplot 1: Surge (e_u & T_u) - No title, all English labels
    line_eu, = ax1.plot([], [], color='#1f77b4', lw=2.0, label='Surge Error $e_u$ [m]')
    line_Tu, = ax1_r.plot([], [], color='#ff7f0e', lw=1.6, alpha=0.85, linestyle='--', label='Surge Force $T_u$ [N]')
    ax1.axhline(0.0, color='gray', lw=0.8, alpha=0.5)
    ax1.set_ylabel('$e_u$ [m]', color='#1f77b4', fontweight='bold')
    ax1_r.set_ylabel('$T_u$ [N]', color='#ff7f0e', fontweight='bold')
    ax1.set_ylim(-1.2, 1.2)
    ax1_r.set_ylim(-25.0, 25.0)
    lines1 = [line_eu, line_Tu]
    ax1.legend(lines1, [l.get_label() for l in lines1], loc='upper right')

    # Subplot 2: Sway (e_v & T_v) - No title, all English labels
    line_ev, = ax2.plot([], [], color='#2ca02c', lw=2.0, label='Sway Error $e_v$ [m]')
    line_Tv, = ax2_r.plot([], [], color='#d62728', lw=1.6, alpha=0.85, linestyle='--', label='Sway Force $T_v$ [N]')
    ax2.axhline(0.0, color='gray', lw=0.8, alpha=0.5)
    ax2.set_ylabel('$e_v$ [m]', color='#2ca02c', fontweight='bold')
    ax2_r.set_ylabel('$T_v$ [N]', color='#d62728', fontweight='bold')
    ax2.set_ylim(-1.2, 1.2)
    ax2_r.set_ylim(-20.0, 20.0)
    lines2 = [line_ev, line_Tv]
    ax2.legend(lines2, [l.get_label() for l in lines2], loc='upper right')

    # Subplot 3: Yaw (e_psi & T_r) - No title, all English labels
    line_epsi, = ax3.plot([], [], color='#9467bd', lw=2.0, label='Yaw Error $e_\\psi$ [rad]')
    line_Tr, = ax3_r.plot([], [], color='#8c564b', lw=1.6, alpha=0.85, linestyle='--', label='Yaw Torque $T_r$ [N·m]')
    ax3.axhline(0.0, color='gray', lw=0.8, alpha=0.5)
    ax3.set_ylabel('$e_\\psi$ [rad]', color='#9467bd', fontweight='bold')
    ax3_r.set_ylabel('$T_r$ [N·m]', color='#8c564b', fontweight='bold')
    ax3.set_xlabel('Time [s]', fontweight='bold')
    ax3.set_ylim(-math.pi, math.pi)
    ax3_r.set_ylim(-12.0, 12.0)
    lines3 = [line_epsi, line_Tr]
    ax3.legend(lines3, [l.get_label() for l in lines3], loc='upper right')

    plt.tight_layout()

    def update_plot(_frame):
        with node.lock:
            if len(node.t_buf) < 2:
                return line_eu, line_Tu, line_ev, line_Tv, line_epsi, line_Tr
            t = np.array(node.t_buf)
            eu = np.array(node.eu_buf)
            ev = np.array(node.ev_buf)
            epsi = np.array(node.epsi_buf)
            Tu = np.array(node.Tu_buf)
            Tv = np.array(node.Tv_buf)
            Tr = np.array(node.Tr_buf)

        line_eu.set_data(t, eu)
        line_Tu.set_data(t, Tu)

        line_ev.set_data(t, ev)
        line_Tv.set_data(t, Tv)

        line_epsi.set_data(t, epsi)
        line_Tr.set_data(t, Tr)

        t_max = max(10.0, float(t[-1]))
        t_min = max(0.0, t_max - node.window_sec) if node.window_sec > 0 else 0.0
        ax3.set_xlim(t_min, t_max + 1.0)

        # Auto-adjust Y limits smoothly if needed
        max_eu = max(0.5, float(np.max(np.abs(eu))) * 1.2)
        ax1.set_ylim(-max_eu, max_eu)
        max_ev = max(0.5, float(np.max(np.abs(ev))) * 1.2)
        ax2.set_ylim(-max_ev, max_ev)

        return line_eu, line_Tu, line_ev, line_Tv, line_epsi, line_Tr

    ani = animation.FuncAnimation(fig, update_plot, interval=150, blit=False, cache_frame_data=False)

    try:
        plt.show()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Save plot on exit if data was recorded
        with node.lock:
            has_samples = len(node.t_buf) > 5
            if has_samples:
                t_max = float(node.t_buf[-1])
                ax3.set_xlim(0.0, max(10.0, t_max))
        if has_samples:
            out_dir = Path("/home/brayan/ros2_ws/src/pladypos/experiments/processed_data")
            out_dir.mkdir(parents=True, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            png_path = out_dir / f"gcs_errors_control_{ts}.png"
            latest_png = out_dir / "gcs_errors_control_latest.png"
            fig.savefig(png_path, dpi=180, bbox_inches='tight')
            fig.savefig(latest_png, dpi=180, bbox_inches='tight')
            node.get_logger().info(f"Saved plot to {latest_png}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

