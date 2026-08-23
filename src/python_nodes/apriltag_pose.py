#!/usr/bin/env python3
"""
apriltag_pose.py
Nodo ROS 2 headless: detecta AprilTag 36h11 ID=285 desde el stream MJPEG
y publica odometría RAW (sin filtrar) con covarianzas calibradas en /usv5/odom
+ TF odom→usv5.  El filtrado lo hace robot_localization (EKF).

Topics publicados:
  /usv5/odom  (nav_msgs/Odometry)   — medición AprilTag con covarianzas
  TF: odom → usv5                   — propagado del odom anterior
"""

import math
import threading
import time
import urllib.request
import json

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# ─── Parámetros ────────────────────────────────────────────────────────────────
STREAM_URL  = "http://10.250.253.1:8083/stream?topic=/camera_0352/camera_0352/image_raw"
CALIB_URL   = "http://10.250.253.1:8083/calibration?camera=0352"
TAG_ID      = 285
TAG_SIDE_M  = 0.25
ARUCO_DICT  = cv2.aruco.DICT_APRILTAG_36h11
ODOM_TOPIC  = "/usv5/odom"
FRAME_ID    = "odom"
CHILD_FRAME = "usv5"

DEFAULT_FX = 1109.2076
DEFAULT_FY = 1114.4800
DEFAULT_CX = 1019.4268
DEFAULT_CY = 1033.4869

# ─── Covarianzas de medición ───────────────────────────────────────────────────
K_XY  = 0.03   # 3 cm por metro de distancia (medición confiable y rápida)
K_YAW = 0.03   # ~1.7° por metro de distancia
COV_Z = 1e6
COV_R = 1e6
COV_P = 1e6
COV_VEL = 1e6  # twist no medido → ignorado por EKF

# Filtro EMA ligero en el nodo para eliminar ruido de píxel con cero lag acumulado
EMA_ALPHA = 0.4

# ─── Calibración ───────────────────────────────────────────────────────────────

def fetch_calibration():
    try:
        with urllib.request.urlopen(CALIB_URL, timeout=3) as r:
            data = json.load(r)
        m  = data["camera_matrix"]["data"]
        fx, cx = float(m[0]), float(m[2])
        fy, cy = float(m[4]), float(m[5])
    except Exception:
        fx, fy, cx, cy = DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY

    K    = np.array([[fx, 0, cx],
                     [0, fy, cy],
                     [0,  0,  1]], dtype=np.float64)
    dist = np.zeros((5, 1), dtype=np.float64)
    return K, dist


# ─── MJPEG reader ──────────────────────────────────────────────────────────────

class MJPEGReader:
    def __init__(self, url: str):
        self._url       = url
        self._frame     = None
        self._lock      = threading.Lock()
        self._stop      = threading.Event()
        self._new_frame = threading.Event()
        self._t         = threading.Thread(target=self._run, daemon=True)

    def start(self):  self._t.start()

    def stop(self):
        self._stop.set()
        self._new_frame.set()
        self._t.join(timeout=2)

    def wait_new_frame(self, timeout=1.0):
        if not self._new_frame.wait(timeout=timeout):
            return None
        self._new_frame.clear()
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def _run(self):
        while not self._stop.is_set():
            try:
                req = urllib.request.urlopen(self._url, timeout=10)
                buf = b""
                while not self._stop.is_set():
                    chunk = req.read(4096)
                    if not chunk:
                        break
                    buf += chunk
                    a  = buf.find(b"\xff\xd8")
                    b_ = buf.find(b"\xff\xd9")
                    if a != -1 and b_ != -1 and b_ > a:
                        jpg = buf[a:b_+2]
                        buf = buf[b_+2:]
                        img = cv2.imdecode(
                            np.frombuffer(jpg, np.uint8), cv2.IMREAD_GRAYSCALE)
                        if img is not None:
                            with self._lock:
                                self._frame = img
                            self._new_frame.set()
            except Exception:
                if not self._stop.is_set():
                    time.sleep(1)


# ─── Detector ArUco ────────────────────────────────────────────────────────────

def build_detector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    try:
        params   = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return detector, None
    except AttributeError:
        params = cv2.aruco.DetectorParameters_create()
        return aruco_dict, params


def detect_tag(detector_info, gray):
    det, params = detector_info
    if params is None:
        corners, ids, _ = det.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, det, parameters=params)
    if ids is None:
        return None
    for i, id_val in enumerate(ids.flatten()):
        if int(id_val) == TAG_ID:
            return corners[i][0]
    return None


# ─── Estimación de pose 6-DOF ──────────────────────────────────────────────────

HALF    = TAG_SIDE_M / 2.0
OBJ_PTS = np.array([
    [-HALF,  HALF, 0],
    [ HALF,  HALF, 0],
    [ HALF, -HALF, 0],
    [-HALF, -HALF, 0],
], dtype=np.float64)

# ─── Convención FLU / ENU ──────────────────────────────────────────────────────
# Cámara en el TECHO mirando ABAJO.  Tag en la superficie del agua mirando ARRIBA.
# solvePnP tvec está en frame de cámara: X=derecha, Y=abajo, Z=profundidad.
#   x_world =  tvec[0]            (eje fijo horizontal)
#   y_world = -tvec[1]            (cam Y apunta abajo → negar para Z-up)
# Yaw FLU: CCW positivo visto desde arriba.
#   En coords de cámara (Y=abajo), CW desde arriba da atan2 creciente,
#   por lo tanto yaw_flu = -atan2(R[1,0], R[0,0]).
#   Offset +π/2 porque el frente del robot coincide con el eje Y del tag.

_YAW_OFFSET = math.pi / 2.0   # frente del robot = tag Y → +90°


def estimate_pose(corners_px, K, dist):
    """
    Retorna (x, y, yaw, depth) en convención FLU/ENU, o
    (None, None, None, None) si falla.

    Posición (en frame fijo de la cámara, plano horizontal):
      x =  tvec[0]
      y = -tvec[1]   (cámara Y apunta abajo, mundo Y apunta "arriba-en-imagen")

    Yaw (FLU, CCW+ visto desde arriba):
      yaw = -atan2(R[1,0], R[0,0]) + π/2
    """
    try:
        ok, rvec, tvec = cv2.solvePnP(
            OBJ_PTS, corners_px.astype(np.float64),
            K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None, None, None, None

        x     =  float(tvec.flat[0])
        y     = -float(tvec.flat[1])          # cam Y abajo → negar
        depth = max(float(tvec.flat[2]), 0.1)

        R, _  = cv2.Rodrigues(rvec)
        yaw   = -math.atan2(float(R[1, 0]), float(R[0, 0])) + _YAW_OFFSET

        return x, y, yaw, depth
    except Exception:
        return None, None, None, None


# ─── Helpers ───────────────────────────────────────────────────────────────────

def yaw_to_quat(yaw: float):
    """Quaternion de rotación pura en Z → (w, x, y, z)."""
    h = yaw * 0.5
    return math.cos(h), 0.0, 0.0, math.sin(h)


def make_pose_cov(depth: float):
    """Covarianza 6×6 diagonal aplanada escalada con la distancia al tag."""
    cov_xy  = (K_XY  * depth) ** 2
    cov_yaw = (K_YAW * depth) ** 2
    c = [0.0] * 36
    for i, v in enumerate([cov_xy, cov_xy, COV_Z, COV_R, COV_P, cov_yaw]):
        c[i * 7] = v
    return c


TWIST_COV = [COV_VEL if i % 7 == 0 else 0.0 for i in range(36)]


# ─── Nodo ROS 2 ────────────────────────────────────────────────────────────────

class AprilTagPoseNode(Node):

    def __init__(self):
        super().__init__("apriltag_pose")
        self._pub_odom = self.create_publisher(Odometry, ODOM_TOPIC, 10)

        self._K, self._dist = fetch_calibration()
        self._detector      = build_detector()
        self._reader        = MJPEGReader(STREAM_URL)
        self._stop          = threading.Event()
        self._thread        = threading.Thread(target=self._loop, daemon=True)

        # Estado filtrado EMA + Mediana para eliminar ruido de alta frecuencia
        self._fx:   float | None = None
        self._fy:   float | None = None
        self._fyaw: float | None = None
        self._buf_x: list[float] = []
        self._buf_y: list[float] = []

    def start(self):
        self._reader.start()
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._reader.stop()
        self._thread.join(timeout=3)

    def _loop(self):
        while not self._stop.is_set():
            gray = self._reader.wait_new_frame(timeout=0.5)
            if gray is None:
                continue
            corners = detect_tag(self._detector, gray)
            if corners is None:
                self._fx = self._fy = self._fyaw = None
                self._buf_x = []
                self._buf_y = []
                continue
            x, y, yaw, depth = estimate_pose(corners, self._K, self._dist)
            if x is None:
                continue

            # ── Filtro de Fuerza Máxima en 5 cm ───────────────────────────────
            # Mantiene el umbral estrictamente en 5 cm (0.05 m).
            # Dentro de los 5 cm, aplica fuerza máxima de filtrado (alpha = 0.005).
            if self._fx is None:
                self._fx, self._fy, self._fyaw = x, y, yaw
                self._buf_x = [x]
                self._buf_y = [y]
            else:
                # 1. Filtro de Mediana (3 muestras) para decapitar picos de ruido
                self._buf_x.append(x)
                self._buf_y.append(y)
                if len(self._buf_x) > 3:
                    self._buf_x.pop(0)
                    self._buf_y.pop(0)
                
                x_med = float(np.median(self._buf_x))
                y_med = float(np.median(self._buf_y))

                # 2. Distancia respecto al umbral estricto de 5 cm
                dist_delta = math.hypot(x_med - self._fx, y_med - self._fy)

                if dist_delta < 0.05:
                    # Fuerza máxima de filtrado dentro de los 5 cm (alpha = 0.005)
                    a = 0.005
                else:
                    # Fuera de los 5 cm: transición rápida a movimiento real
                    ratio = (dist_delta - 0.05) / 0.10
                    a = min(0.85, 0.05 + ratio * 0.80)

                self._fx = a * x_med + (1.0 - a) * self._fx
                self._fy = a * y_med + (1.0 - a) * self._fy

                # 3. Filtrado suave en orientación (yaw)
                dyaw = (yaw - self._fyaw + math.pi) % (2 * math.pi) - math.pi
                # a_yaw = 0.08 elimina el temblor angular sin bloquear ni dar tirones
                a_yaw = 0.08 if abs(dyaw) < 0.05 else 0.60
                self._fyaw = self._fyaw + a_yaw * dyaw

            if rclpy.ok():
                self._publish(self._fx, self._fy, self._fyaw, depth)

    def _publish(self, x: float, y: float, yaw: float, depth: float):
        stamp = self.get_clock().now().to_msg()
        qw, qx, qy, qz = yaw_to_quat(yaw)

        # ── Odometry ──────────────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = FRAME_ID
        odom.child_frame_id  = CHILD_FRAME

        odom.pose.pose.position.x    = x
        odom.pose.pose.position.y    = y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz

        # Covarianza escalada con la distancia real al tag
        odom.pose.covariance  = make_pose_cov(depth)
        odom.twist.covariance = TWIST_COV

        self._pub_odom.publish(odom)
        # TF publicado por robot_localization (odom → usv5, filtrado)


# ─── Entrypoint ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPoseNode()
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
