#!/usr/bin/env python3
"""
record_orientation.py
─────────────────────
Graba la orientación (quaternion completo + yaw) de dos fuentes:
  • AprilTag  → /usv5/odom          (nav_msgs/Odometry)
  • IMU       → /usv5/imu/data      (sensor_msgs/Imu)

Al cerrar con Ctrl+C guarda un CSV en ~/orientation_log.csv listo para
pasarle al script analyze_orientation.py.

Uso:
  ros2 run pladypos record_orientation.py
  # mueve el robot ~30 s con giros amplios
  # Ctrl+C  →  se genera el CSV
"""

import math
import csv
import pathlib
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


OUTPUT_FILE  = pathlib.Path.home() / "orientation_log.csv"
TAG_TOPIC    = "/usv5/odom"
IMU_TOPIC    = "/usv5/imu/data"

# Ventana de tiempo para emparejar mensajes TAG ↔ IMU (segundos)
SYNC_WINDOW  = 0.15


def quat_to_yaw(x, y, z, w):
    """Extrae yaw (rotación en Z) de un quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RecordOrientationNode(Node):

    def __init__(self):
        super().__init__("record_orientation")

        # Buffers de mensajes sin emparejar
        self._tag_buf: list[dict] = []   # {t, qx, qy, qz, qw, yaw}
        self._imu_buf: list[dict] = []   # {t, qx, qy, qz, qw, yaw}

        # Filas emparejadas ya listas para guardar
        self._rows: list[dict] = []

        self.create_subscription(Odometry, TAG_TOPIC, self._tag_cb, 20)
        self.create_subscription(Imu,      IMU_TOPIC, self._imu_cb, 20)

        self.get_logger().info(
            f"▶ Grabando orientaciones…\n"
            f"  TAG  ← {TAG_TOPIC}\n"
            f"  IMU  ← {IMU_TOPIC}\n"
            f"  Mueve el robot con giros amplios y luego Ctrl+C"
        )

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _tag_cb(self, msg: Odometry):
        t  = self._stamp_to_sec(msg.header.stamp)
        q  = msg.pose.pose.orientation
        entry = dict(t=t, qx=q.x, qy=q.y, qz=q.z, qw=q.w,
                     yaw=quat_to_yaw(q.x, q.y, q.z, q.w))
        self._tag_buf.append(entry)
        self._try_sync()

    def _imu_cb(self, msg: Imu):
        t  = self._stamp_to_sec(msg.header.stamp)
        q  = msg.orientation
        entry = dict(t=t, qx=q.x, qy=q.y, qz=q.z, qw=q.w,
                     yaw=quat_to_yaw(q.x, q.y, q.z, q.w))
        self._imu_buf.append(entry)
        self._try_sync()

    # ── Sincronización temporal simple (nearest-neighbor) ──────────────────────

    def _try_sync(self):
        """Para cada mensaje TAG reciente, busca el IMU más cercano en tiempo."""
        matched_tag_indices = []
        for i, tag in enumerate(self._tag_buf):
            best_dt = SYNC_WINDOW + 1.0
            best_imu = None
            for imu in self._imu_buf:
                dt = abs(tag["t"] - imu["t"])
                if dt < best_dt:
                    best_dt = dt
                    best_imu = imu
            if best_imu is not None and best_dt <= SYNC_WINDOW:
                self._rows.append({
                    "t":          tag["t"],
                    # TAG quaternion
                    "tag_qx":     tag["qx"],
                    "tag_qy":     tag["qy"],
                    "tag_qz":     tag["qz"],
                    "tag_qw":     tag["qw"],
                    "tag_yaw":    tag["yaw"],
                    # IMU quaternion
                    "imu_qx":     best_imu["qx"],
                    "imu_qy":     best_imu["qy"],
                    "imu_qz":     best_imu["qz"],
                    "imu_qw":     best_imu["qw"],
                    "imu_yaw":    best_imu["yaw"],
                    # Diferencia de tiempo del par
                    "dt_sync_s":  round(abs(tag["t"] - best_imu["t"]), 4),
                })
                matched_tag_indices.append(i)

        # Eliminar los TAG ya emparejados del buffer
        for i in sorted(matched_tag_indices, reverse=True):
            self._tag_buf.pop(i)

        # Limpiar buffer IMU de mensajes muy viejos (> 2 s)
        if self._imu_buf:
            t_last = self._imu_buf[-1]["t"]
            self._imu_buf = [e for e in self._imu_buf if t_last - e["t"] < 2.0]

    # ── Utilidades ─────────────────────────────────────────────────────────────

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def save_csv(self):
        if not self._rows:
            self.get_logger().warn("No se capturaron pares sincronizados. CSV no guardado.")
            return

        fieldnames = [
            "t",
            "tag_qx", "tag_qy", "tag_qz", "tag_qw", "tag_yaw",
            "imu_qx", "imu_qy", "imu_qz", "imu_qw", "imu_yaw",
            "dt_sync_s",
        ]
        with open(OUTPUT_FILE, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._rows)

        self.get_logger().info(
            f"✅ {len(self._rows)} pares guardados en {OUTPUT_FILE}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RecordOrientationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
