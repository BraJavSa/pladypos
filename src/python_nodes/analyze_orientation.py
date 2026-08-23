#!/usr/bin/env python3
"""
analyze_orientation.py
───────────────────────
Analiza el CSV generado por record_orientation.py para encontrar la
rotación que hace que el yaw del tag coincida con el yaw de la IMU.

La IMU es la referencia (ground truth). El tag puede estar rotado en
cualquier eje respecto al frame de la IMU.

Uso:
  python3 analyze_orientation.py ~/orientation_log.csv

Salida:
  • Gráficas de yaw TAG vs IMU (antes y después de corrección)
  • Offset de yaw en grados y radianes
  • Quaternion de rotación correctora R_tag→imu
  • Código Python listo para pegar en apriltag_pose.py
"""

import sys
import math
import pathlib

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from scipy.spatial.transform import Rotation


# ─── Utilidades ────────────────────────────────────────────────────────────────

def wrap_pi(angle):
    """Envuelve ángulo al rango (-π, π]."""
    return (np.asarray(angle) + np.pi) % (2 * np.pi) - np.pi


def quat_cols_to_rotation(df, prefix):
    """
    Lee columnas {prefix}_qx/qy/qz/qw del DataFrame y devuelve un array
    de objetos Rotation de scipy (N,).
    """
    q = df[[f"{prefix}_qx", f"{prefix}_qy",
            f"{prefix}_qz", f"{prefix}_qw"]].values          # (N,4) — x,y,z,w
    return Rotation.from_quat(q)                              # scipy: (x,y,z,w)


# ─── Carga de datos ────────────────────────────────────────────────────────────

def load(csv_path: str) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    required = {"t", "tag_yaw", "imu_yaw",
                "tag_qx", "tag_qy", "tag_qz", "tag_qw",
                "imu_qx", "imu_qy", "imu_qz", "imu_qw"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"Columnas faltantes en el CSV: {missing}")
    print(f"✅  {len(df)} pares cargados  |  dt_sync medio: "
          f"{df['dt_sync_s'].mean()*1000:.1f} ms")
    return df


# ─── Análisis 1: offset de yaw simple (modelo 2D) ─────────────────────────────

def analyze_yaw_offset(df: pd.DataFrame):
    """
    Calcula el offset de yaw medio entre TAG e IMU usando la diferencia
    circular (wraparound-safe).  Modelo: yaw_imu ≈ yaw_tag + offset
    """
    diff = wrap_pi(df["imu_yaw"].values - df["tag_yaw"].values)
    offset_rad = float(np.median(diff))           # mediana → robusto a outliers
    offset_deg = math.degrees(offset_rad)

    # Residuo después de corregir
    corrected_tag_yaw = wrap_pi(df["tag_yaw"].values + offset_rad)
    residuals = wrap_pi(df["imu_yaw"].values - corrected_tag_yaw)
    rmse_deg = math.degrees(float(np.sqrt(np.mean(residuals ** 2))))

    print("\n" + "═"*60)
    print("  ANÁLISIS DE YAW (2D)")
    print("═"*60)
    print(f"  Offset mediano  :  {offset_rad:.6f} rad  ({offset_deg:.3f}°)")
    print(f"  RMSE residual   :  {math.degrees(float(np.std(residuals))):.3f}°  "
          f"(1σ)   |   RMSE={rmse_deg:.3f}°")
    print("═"*60)

    return offset_rad, offset_deg, corrected_tag_yaw, residuals


# ─── Análisis 2: rotación 3D (quaternion) ─────────────────────────────────────

def analyze_3d_rotation(df: pd.DataFrame):
    """
    Encuentra la rotación R tal que:
        R_imu ≈ R_correction · R_tag
    ⟹  R_correction = R_imu · R_tag⁻¹

    Usa Wahba / promediado de quaterniones para obtener la corrección media.
    """
    R_tag = quat_cols_to_rotation(df, "tag")
    R_imu = quat_cols_to_rotation(df, "imu")

    # R_correction[i] = R_imu[i] · R_tag[i]⁻¹
    R_corr_per_sample = R_imu * R_tag.inv()

    # Promediado de quaterniones (Markley et al.)
    quats = R_corr_per_sample.as_quat()          # (N, 4) — x,y,z,w

    # Asegurar hemisferio consistente (evitar promedio de q y -q)
    ref = quats[0]
    signs = np.sign(np.dot(quats, ref))
    signs[signs == 0] = 1
    quats = quats * signs[:, None]

    mean_quat = quats.mean(axis=0)
    mean_quat /= np.linalg.norm(mean_quat)       # normalizar

    R_mean = Rotation.from_quat(mean_quat)
    euler_deg = R_mean.as_euler("xyz", degrees=True)
    euler_rad = R_mean.as_euler("xyz", degrees=False)

    print("\n" + "═"*60)
    print("  ANÁLISIS 3D — ROTACIÓN CORRECTORA (R_tag → IMU)")
    print("═"*60)
    print(f"  Quaternion  (x,y,z,w): {mean_quat.round(6)}")
    print(f"  Euler XYZ (grados)   : roll={euler_deg[0]:.3f}°  "
          f"pitch={euler_deg[1]:.3f}°  yaw={euler_deg[2]:.3f}°")
    print(f"  Euler XYZ (rad)      : roll={euler_rad[0]:.6f}  "
          f"pitch={euler_rad[1]:.6f}  yaw={euler_rad[2]:.6f}")
    print("═"*60)

    # ── Verificación: residuos 3D ────────────────────────────────────────────
    R_tag_corrected = R_mean * R_tag
    R_diff = R_imu * R_tag_corrected.inv()
    euler_res = R_diff.as_euler("xyz", degrees=True)
    rmse_roll  = float(np.sqrt(np.mean(euler_res[:, 0]**2)))
    rmse_pitch = float(np.sqrt(np.mean(euler_res[:, 1]**2)))
    rmse_yaw   = float(np.sqrt(np.mean(euler_res[:, 2]**2)))
    print(f"  RMSE tras corrección : roll={rmse_roll:.3f}°  "
          f"pitch={rmse_pitch:.3f}°  yaw={rmse_yaw:.3f}°")
    print("═"*60)

    return R_mean, mean_quat, euler_deg, euler_rad


# ─── Código listo para pegar ───────────────────────────────────────────────────

def print_patch(offset_rad: float, R_mean: Rotation, euler_rad: np.ndarray):
    quat = R_mean.as_quat()   # x,y,z,w
    print("\n" + "═"*60)
    print("  CÓDIGO PARA PEGAR EN apriltag_pose.py")
    print("═"*60)

    print("""
# ── Corrección de orientación TAG → IMU (calculada offline) ────────────────
import numpy as np
from scipy.spatial.transform import Rotation as _R
""")
    print(f"_R_CORR = _R.from_euler('xyz', [{euler_rad[0]:.8f}, "
          f"{euler_rad[1]:.8f}, {euler_rad[2]:.8f}])  "
          f"# roll={math.degrees(euler_rad[0]):.2f}° "
          f"pitch={math.degrees(euler_rad[1]):.2f}° "
          f"yaw={math.degrees(euler_rad[2]):.2f}°")
    print("""
# En estimate_pose(), después de obtener rvec de solvePnP:
#   R_tag   = cv2.Rodrigues(rvec)[0]
#   R_fix   = _R_CORR.as_matrix() @ R_tag
#   yaw_frd = math.atan2(R_fix[1,0], R_fix[0,0])
""")
    print("═"*60)


# ─── Gráficas ──────────────────────────────────────────────────────────────────

def plot_results(df: pd.DataFrame, offset_rad: float,
                 corrected_tag_yaw: np.ndarray, residuals: np.ndarray):

    t = df["t"].values - df["t"].values[0]   # tiempo relativo en segundos
    imu_yaw = df["imu_yaw"].values
    tag_yaw = df["tag_yaw"].values

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle("Análisis de Orientación: AprilTag vs IMU", fontsize=14, fontweight="bold")
    gs  = gridspec.GridSpec(3, 2, figure=fig, hspace=0.45, wspace=0.35)

    # ── 1. Yaw raw ───────────────────────────────────────────────────────────
    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot(t, np.degrees(imu_yaw), label="IMU yaw (ref)",  color="#2ecc71", lw=1.5)
    ax1.plot(t, np.degrees(tag_yaw), label="TAG yaw (raw)",  color="#e74c3c", lw=1.0, alpha=0.7)
    ax1.set_title("Yaw crudo TAG vs IMU")
    ax1.set_xlabel("Tiempo (s)")
    ax1.set_ylabel("Yaw (°)")
    ax1.legend()
    ax1.grid(alpha=0.3)

    # ── 2. Yaw corregido ─────────────────────────────────────────────────────
    ax2 = fig.add_subplot(gs[1, :])
    ax2.plot(t, np.degrees(imu_yaw),           label="IMU yaw (ref)",       color="#2ecc71", lw=1.5)
    ax2.plot(t, np.degrees(corrected_tag_yaw), label="TAG yaw (corregido)", color="#3498db", lw=1.0, alpha=0.85)
    ax2.set_title(f"Yaw TAG corregido  (offset = {math.degrees(offset_rad):.2f}°)")
    ax2.set_xlabel("Tiempo (s)")
    ax2.set_ylabel("Yaw (°)")
    ax2.legend()
    ax2.grid(alpha=0.3)

    # ── 3. Histograma residuos ───────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[2, 0])
    ax3.hist(np.degrees(residuals), bins=40, color="#9b59b6", edgecolor="white", alpha=0.8)
    ax3.axvline(0, color="white", ls="--", lw=1)
    ax3.set_title("Residuos tras corrección (°)")
    ax3.set_xlabel("Error (°)")
    ax3.set_ylabel("Frecuencia")
    ax3.grid(alpha=0.3)

    # ── 4. TAG vs IMU scatter ────────────────────────────────────────────────
    ax4 = fig.add_subplot(gs[2, 1])
    ax4.scatter(np.degrees(imu_yaw), np.degrees(tag_yaw),
                s=4, alpha=0.4, color="#e67e22", label="raw")
    ax4.scatter(np.degrees(imu_yaw), np.degrees(corrected_tag_yaw),
                s=4, alpha=0.4, color="#3498db", label="corregido")
    lims = [min(np.degrees(imu_yaw).min(), np.degrees(tag_yaw).min()) - 5,
            max(np.degrees(imu_yaw).max(), np.degrees(tag_yaw).max()) + 5]
    ax4.plot(lims, lims, "w--", lw=1, label="ideal")
    ax4.set_xlim(lims); ax4.set_ylim(lims)
    ax4.set_title("TAG vs IMU (scatter)")
    ax4.set_xlabel("IMU yaw (°)")
    ax4.set_ylabel("TAG yaw (°)")
    ax4.legend(markerscale=3)
    ax4.grid(alpha=0.3)

    fig.patch.set_facecolor("#1a1a2e")
    for ax in fig.get_axes():
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="white")
        ax.xaxis.label.set_color("white")
        ax.yaxis.label.set_color("white")
        ax.title.set_color("white")
        for sp in ax.spines.values():
            sp.set_edgecolor("#444")

    out = pathlib.Path("orientation_analysis.png")
    plt.savefig(out, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"\n📊 Gráfica guardada en: {out.resolve()}")
    plt.show()


# ─── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Uso: python3 analyze_orientation.py <ruta_al_csv>")
        sys.exit(1)

    csv_path = sys.argv[1]
    df = load(csv_path)

    # 1. Offset de yaw 2D
    offset_rad, offset_deg, corrected_yaw, residuals = analyze_yaw_offset(df)

    # 2. Rotación 3D completa
    R_mean, mean_quat, euler_deg, euler_rad = analyze_3d_rotation(df)

    # 3. Código listo para pegar
    print_patch(offset_rad, R_mean, euler_rad)

    # 4. Gráficas
    plot_results(df, offset_rad, corrected_yaw, residuals)


if __name__ == "__main__":
    main()
