#!/usr/bin/env python3
"""
apriltag_pose.py
Nodo ROS 2 headless para detección de AprilTag 36h11 ID=285 desde stream MJPEG.
Calcula la pose 6D respecto al marco de la cámara (frame top/padre) usando solvePnP
y publica Odometría (nav_msgs/Odometry) en /usv5/odom a 20 Hz ininterrumpidos.
Publica la transformada TF directa (camera -> usv5), siendo la cámara el marco raíz (top frame).
"""

import json
import math
import threading
import time
import urllib.request
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# ─── Parámetros por defecto ───────────────────────────────────────────────────
DEFAULT_STREAM_URL         = "http://10.250.253.1:8083/stream?topic=/camera_0354/camera_0354/image_raw"
DEFAULT_CALIB_URL          = "http://10.250.253.1:8083/calibration?camera=0354"
DEFAULT_TAG_ID             = 285
DEFAULT_TAG_SIZE           = 0.25
DEFAULT_ODOM_TOPIC         = "/usv5/odom"
DEFAULT_TAG_DETECTED_TOPIC = "/usv5/tag_detected"
DEFAULT_CAMERA_FRAME       = "camera"
DEFAULT_USV_FRAME          = "usv5"
DEFAULT_RATE_HZ            = 10.0

DEFAULT_FX = 1109.2076
DEFAULT_FY = 1114.4800
DEFAULT_CX = 1019.4268
DEFAULT_CY = 1033.4869


def fetch_calibration(calib_url: str):
    """Obtiene la matriz de calibración K desde la URL o usa valores por defecto."""
    try:
        with urllib.request.urlopen(calib_url, timeout=3) as r:
            data = json.load(r)
        m = data["camera_matrix"]["data"]
        fx, cx = float(m[0]), float(m[2])
        fy, cy = float(m[4]), float(m[5])
    except Exception:
        fx, fy, cx, cy = DEFAULT_FX, DEFAULT_FY, DEFAULT_CX, DEFAULT_CY

    K = np.array([[fx, 0, cx],
                  [0, fy, cy],
                  [0,  0,  1]], dtype=np.float64)
    dist = np.zeros((5, 1), dtype=np.float64)
    return K, dist


class MJPEGReader:
    """Lector asíncrono del stream MJPEG HTTP."""
    def __init__(self, url: str):
        self._url = url
        self._frame = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._new_frame = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._t.start()

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
                    a = buf.find(b"\xff\xd8")
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


def build_detector():
    """Construye el detector ArUco/AprilTag compatible con distintas versiones de OpenCV."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    try:
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return detector, None
    except AttributeError:
        params = cv2.aruco.DetectorParameters_create()
        return aruco_dict, params


def detect_tag(detector_info, gray, target_id: int):
    """Detecta el AprilTag con ID especificado y retorna sus esquinas en la imagen."""
    det, params = detector_info
    if params is None:
        corners, ids, _ = det.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, det, parameters=params)
    if ids is None:
        return None
    for i, id_val in enumerate(ids.flatten()):
        if int(id_val) == target_id:
            return corners[i][0]
    return None


def rotation_matrix_to_quaternion(R: np.ndarray):
    """Convierte matriz de rotación 3x3 R a cuaternión normalizado (w, x, y, z)."""
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    if norm < 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    return qw / norm, qx / norm, qy / norm, qz / norm


def estimate_pose(corners_px: np.ndarray, tag_size: float, K: np.ndarray, dist: np.ndarray):
    """
    Calcula la posición 3D (x, y, z) y la orientación en cuaternión (qw, qx, qy, qz)
    del AprilTag con respecto al sistema de coordenadas de la cámara.
    """
    half = tag_size / 2.0
    obj_pts = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0]
    ], dtype=np.float64)

    try:
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, corners_px.astype(np.float64),
            K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None

        x = float(tvec.flat[0])
        y = float(tvec.flat[1])
        z = float(tvec.flat[2])

        R, _ = cv2.Rodrigues(rvec)
        qw, qx, qy, qz = rotation_matrix_to_quaternion(R)

        return x, y, z, qw, qx, qy, qz
    except Exception:
        return None


def make_pose_covariance(distance: float):
    """Matriz de covarianza de pose 6x6 escalada con la distancia al tag."""
    var_pos = max(0.001, 0.01 * (distance ** 2))
    var_rot = max(0.005, 0.03 * (distance ** 2))
    cov = [0.0] * 36
    cov[0]  = var_pos  # x
    cov[7]  = var_pos  # y
    cov[14] = var_pos  # z
    cov[21] = var_rot  # roll
    cov[28] = var_rot  # pitch
    cov[35] = var_rot  # yaw
    return cov


def make_twist_covariance():
    """Matriz de covarianza de velocidad 6x6."""
    cov = [0.0] * 36
    cov[0]  = 0.05  # vx
    cov[7]  = 0.05  # vy
    cov[14] = 0.05  # vz
    cov[21] = 0.1   # wx
    cov[28] = 0.1   # wy
    cov[35] = 0.1   # wz
    return cov


class MovingAverageFilter2D:
    """Filtro de media móvil 2D con tamaño de ventana configurable."""
    def __init__(self, window_size: int = 6):
        self.window_size = window_size
        self.buf_x = deque(maxlen=window_size)
        self.buf_y = deque(maxlen=window_size)

    def reset(self):
        self.buf_x.clear()
        self.buf_y.clear()

    def update(self, x: float, y: float):
        self.buf_x.append(x)
        self.buf_y.append(y)
        x_avg = sum(self.buf_x) / len(self.buf_x)
        y_avg = sum(self.buf_y) / len(self.buf_y)
        return x_avg, y_avg


class AlphaBeta1D:
    """Filtro Alpha-Beta 1D para suavizado y estimación de velocidad."""
    def __init__(self, alpha: float = 0.63, beta: float = 0.37):
        self.alpha = alpha
        self.beta = beta
        self.x = None
        self.v = 0.0

    def reset(self):
        self.x = None
        self.v = 0.0

    def update(self, z: float, dt: float):
        if self.x is None:
            self.x = z
            self.v = 0.0
            return self.x, self.v

        x_pred = self.x + self.v * dt
        v_pred = self.v

        res = z - x_pred
        self.x = x_pred + self.alpha * res
        self.v = v_pred + (self.beta / dt) * res

        return self.x, self.v


class AprilTagPoseNode(Node):
    def __init__(self):
        super().__init__("apriltag_pose")

        # Declarar parámetros ROS 2
        self.declare_parameter("stream_url", DEFAULT_STREAM_URL)
        self.declare_parameter("calib_url", DEFAULT_CALIB_URL)
        self.declare_parameter("tag_id", DEFAULT_TAG_ID)
        self.declare_parameter("tag_size", DEFAULT_TAG_SIZE)
        self.declare_parameter("odom_topic", DEFAULT_ODOM_TOPIC)
        self.declare_parameter("tag_detected_topic", DEFAULT_TAG_DETECTED_TOPIC)
        self.declare_parameter("camera_frame_id", DEFAULT_CAMERA_FRAME)
        self.declare_parameter("usv_frame_id", DEFAULT_USV_FRAME)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate", DEFAULT_RATE_HZ)
        self.declare_parameter("use_filter", True)
        self.declare_parameter("window_size", 6)
        self.declare_parameter("ab_alpha", 0.63)
        self.declare_parameter("ab_beta", 0.37)

        self.stream_url = self.get_parameter("stream_url").value
        self.calib_url = self.get_parameter("calib_url").value
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.tag_size = float(self.get_parameter("tag_size").value)
        self.odom_topic = self.get_parameter("odom_topic").value
        self.tag_detected_topic = self.get_parameter("tag_detected_topic").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.usv_frame_id = self.get_parameter("usv_frame_id").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.use_filter = bool(self.get_parameter("use_filter").value)
        self.window_size = int(self.get_parameter("window_size").value)
        self.ab_alpha = float(self.get_parameter("ab_alpha").value)
        self.ab_beta = float(self.get_parameter("ab_beta").value)

        self._pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self._pub_tag_detected = self.create_publisher(Bool, self.tag_detected_topic, 10)
        self._tf_br = TransformBroadcaster(self)

        self._K, self._dist = fetch_calibration(self.calib_url)
        self._detector = build_detector()
        self._reader = MJPEGReader(self.stream_url)

        self._stop = threading.Event()
        self._detection_thread = threading.Thread(target=self._detection_loop, daemon=True)

        self.ma_filter = MovingAverageFilter2D(window_size=self.window_size)
        self.ab_x = AlphaBeta1D(alpha=self.ab_alpha, beta=self.ab_beta)
        self.ab_y = AlphaBeta1D(alpha=self.ab_alpha, beta=self.ab_beta)
        self._last_detection_time = None
        self._prev_x = 0.0
        self._prev_y = 0.0

        self.get_logger().info(
            f"Nodo apriltag_pose (Publicación en tiempo real + Filtro en cascada MA(N={self.window_size}) -> AlphaBeta):\n"
            f"  Topic Odometría: {self.odom_topic}\n"
            f"  Topic Detección Tag: {self.tag_detected_topic}\n"
            f"  Filtro XY: {'Activado' if self.use_filter else 'Desactivado'} (MA_window={self.window_size}, alpha={self.ab_alpha}, beta={self.ab_beta})\n"
            f"  TF: {self.camera_frame_id} (top frame) -> {self.usv_frame_id}\n"
            f"  Tag ID: {self.tag_id} ({self.tag_size} m)"
        )

    def start(self):
        self._reader.start()
        self._detection_thread.start()

    def stop(self):
        self._stop.set()
        self._reader.stop()
        self._detection_thread.join(timeout=3)

    def _detection_loop(self):
        """Hilo dedicado a procesar los fotogramas del MJPEG stream."""
        while not self._stop.is_set():
            gray = self._reader.wait_new_frame(timeout=0.5)
            if gray is None:
                continue

            corners = detect_tag(self._detector, gray, self.tag_id)
            if corners is None:
                continue

            res = estimate_pose(corners, self.tag_size, self._K, self._dist)
            if res is None:
                continue

            x, y, z, qw, qx, qy, qz = res
            z = 5.1
            now_sec = time.time()

            if self.use_filter:
                if self._last_detection_time is None or (now_sec - self._last_detection_time) > 0.5:
                    self.ma_filter.reset()
                    self.ab_x.reset()
                    self.ab_y.reset()
                    dt = 0.1
                else:
                    dt = max(0.001, now_sec - self._last_detection_time)

                # 1. Media móvil de N=6 muestras
                x_avg, y_avg = self.ma_filter.update(x, y)

                # 2. Filtro Alpha-Beta sobre la media móvil
                x_filt, vx = self.ab_x.update(x_avg, dt)
                y_filt, vy = self.ab_y.update(y_avg, dt)
            else:
                x_filt, y_filt = x, y
                vx, vy = 0.0, 0.0
                if self._last_detection_time is not None:
                    dt = now_sec - self._last_detection_time
                    if dt > 0.001:
                        vx = (x - self._prev_x) / dt
                        vy = (y - self._prev_y) / dt
                self._prev_x = x
                self._prev_y = y

            vz = 0.0
            self._last_detection_time = now_sec

            # Publicar ÚNICAMENTE cuando se detecta el tag con éxito
            self._publish_data(x_filt, y_filt, z, qw, qx, qy, qz, vx, vy, vz)

    def _publish_tf(self, stamp, x: float, y: float, z: float, qw: float, qx: float, qy: float, qz: float):
        """Publica la transformada TF directa de la Cámara al USV (camera -> usv5)."""
        if not self.publish_tf:
            return

        t_usv = TransformStamped()
        t_usv.header.stamp = stamp
        t_usv.header.frame_id = self.camera_frame_id
        t_usv.child_frame_id = self.usv_frame_id
        t_usv.transform.translation.x = x
        t_usv.transform.translation.y = y
        t_usv.transform.translation.z = z
        t_usv.transform.rotation.w = qw
        t_usv.transform.rotation.x = qx
        t_usv.transform.rotation.y = qy
        t_usv.transform.rotation.z = qz

        self._tf_br.sendTransform(t_usv)

    def _publish_data(self, x: float, y: float, z: float, qw: float, qx: float, qy: float, qz: float, vx: float, vy: float, vz: float):
        """Publica Odometría, TF y estado de detección al haber una detección válida."""
        stamp = self.get_clock().now().to_msg()

        # 1. Publicar nav_msgs/Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.camera_frame_id
        odom_msg.child_frame_id = self.usv_frame_id

        dist = math.sqrt(x * x + y * y + z * z)
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z

        odom_msg.pose.pose.orientation.w = qw
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz

        odom_msg.pose.covariance = make_pose_covariance(dist)

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = vz
        odom_msg.twist.covariance = make_twist_covariance()

        self._pub_odom.publish(odom_msg)

        # 2. Publicar estado de detección del tag (std_msgs/Bool = True)
        tag_detected_msg = Bool()
        tag_detected_msg.data = True
        self._pub_tag_detected.publish(tag_detected_msg)

        # 3. Publicar TF (camera -> usv5)
        self._publish_tf(stamp, x, y, z, qw, qx, qy, qz)


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