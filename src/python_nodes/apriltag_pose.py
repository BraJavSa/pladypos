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

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# ─── Parámetros por defecto ───────────────────────────────────────────────────
DEFAULT_STREAM_URL   = "http://10.250.253.1:8083/stream?topic=/camera_0352/camera_0352/image_raw"
DEFAULT_CALIB_URL    = "http://10.250.253.1:8083/calibration?camera=0352"
DEFAULT_TAG_ID       = 285
DEFAULT_TAG_SIZE     = 0.25
DEFAULT_ODOM_TOPIC   = "/usv5/odom"
DEFAULT_CAMERA_FRAME = "camera"
DEFAULT_USV_FRAME    = "usv5"
DEFAULT_RATE_HZ      = 20.0

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


class AprilTagPoseNode(Node):
    def __init__(self):
        super().__init__("apriltag_pose")

        # Declarar parámetros ROS 2
        self.declare_parameter("stream_url", DEFAULT_STREAM_URL)
        self.declare_parameter("calib_url", DEFAULT_CALIB_URL)
        self.declare_parameter("tag_id", DEFAULT_TAG_ID)
        self.declare_parameter("tag_size", DEFAULT_TAG_SIZE)
        self.declare_parameter("odom_topic", DEFAULT_ODOM_TOPIC)
        self.declare_parameter("camera_frame_id", DEFAULT_CAMERA_FRAME)
        self.declare_parameter("usv_frame_id", DEFAULT_USV_FRAME)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_rate", DEFAULT_RATE_HZ)

        self.stream_url = self.get_parameter("stream_url").value
        self.calib_url = self.get_parameter("calib_url").value
        self.tag_id = int(self.get_parameter("tag_id").value)
        self.tag_size = float(self.get_parameter("tag_size").value)
        self.odom_topic = self.get_parameter("odom_topic").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.usv_frame_id = self.get_parameter("usv_frame_id").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self._pub_odom = self.create_publisher(Odometry, self.odom_topic, 10)
        self._tf_br = TransformBroadcaster(self)

        self._K, self._dist = fetch_calibration(self.calib_url)
        self._detector = build_detector()
        self._reader = MJPEGReader(self.stream_url)

        self._stop = threading.Event()
        self._detection_thread = threading.Thread(target=self._detection_loop, daemon=True)

        # Estado de última pose detectada y velocidad
        self._lock = threading.Lock()
        self._latest_data = None  # (x, y, z, qw, qx, qy, qz, timestamp, vx, vy, vz)
        self._prev_pose = None    # (x, y, z, timestamp)

        # Timer a frecuencia fija (20 Hz por defecto)
        timer_period = 1.0 / self.publish_rate
        self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f"Nodo apriltag_pose (Odometría + TF a {self.publish_rate} Hz):\n"
            f"  Topic Odometría: {self.odom_topic}\n"
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
            now_sec = time.time()

            vx, vy, vz = 0.0, 0.0, 0.0
            if self._prev_pose is not None:
                px, py, pz, pt = self._prev_pose
                dt = now_sec - pt
                if dt > 0.001:
                    vx = (x - px) / dt
                    vy = (y - py) / dt
                    vz = (z - pz) / dt

            self._prev_pose = (x, y, z, now_sec)

            with self._lock:
                self._latest_data = (x, y, z, qw, qx, qy, qz, now_sec, vx, vy, vz)

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

    def _timer_callback(self):
        """Callback del temporizador ROS 2 ejecutado a 20 Hz fijo sin interrupción."""
        with self._lock:
            data = self._latest_data

        stamp = self.get_clock().now().to_msg()
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.camera_frame_id
        odom_msg.child_frame_id = self.usv_frame_id

        # Si no hay detección o los datos son mayores a 1.0s, publicar odometría y TF vacías (cero)
        if data is None or (time.time() - data[7]) > 1.0:
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0

            odom_msg.pose.pose.orientation.w = 1.0
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = 0.0

            odom_msg.pose.covariance = [0.0] * 36

            odom_msg.twist.twist.linear.x = 0.0
            odom_msg.twist.twist.linear.y = 0.0
            odom_msg.twist.twist.linear.z = 0.0
            odom_msg.twist.twist.angular.x = 0.0
            odom_msg.twist.twist.angular.y = 0.0
            odom_msg.twist.twist.angular.z = 0.0
            odom_msg.twist.covariance = [0.0] * 36

            self._pub_odom.publish(odom_msg)

            # Publicar TF vacía (camera -> usv5 en 0,0,0)
            self._publish_tf(stamp, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
            return

        # Si hay detección activa y reciente:
        x, y, z, qw, qx, qy, qz, ts, vx, vy, vz = data
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

        # Publicar TF de la pose estimada (camera -> usv5)
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
