#!/usr/bin/env python3

import os
os.environ.pop("ROS_DISCOVERY_SERVER", None)

import sys
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
import apriltag_msgs.msg
import tf2_ros
import pupil_apriltags

def rot_matrix_to_quaternion(R):
    trace = R[0][0] + R[1][1] + R[2][2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2][1] - R[1][2]) * s
        y = (R[0][2] - R[2][0]) * s
        z = (R[1][0] - R[0][1]) * s
    elif (R[0][0] > R[1][1]) and (R[0][0] > R[2][2]):
        s = 2.0 * np.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2])
        w = (R[2][1] - R[1][2]) / s
        x = 0.25 * s
        y = (R[0][1] + R[1][0]) / s
        z = (R[0][2] + R[2][0]) / s
    elif R[1][1] > R[2][2]:
        s = 2.0 * np.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2])
        w = (R[0][2] - R[2][0]) / s
        x = (R[0][1] + R[1][0]) / s
        y = 0.25 * s
        z = (R[1][2] + R[2][1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1])
        w = (R[1][0] - R[0][1]) / s
        x = (R[0][2] + R[2][0]) / s
        y = (R[1][2] + R[2][1]) / s
        z = 0.25 * s
    return w, x, y, z

class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        # Declarar todos los parámetros
        self.declare_parameter('image_topic', '/camera_0354/image_raw/compressed')
        self.declare_parameter('camera_info_topic', '/camera_0354/camera_info')
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_size', 0.16)  # tamaño en metros
        self.declare_parameter('axis_scale', 0.25)  # escala del eje corto (25% del tag)
        self.declare_parameter('quad_decimate', 1.0)  # 1.0 para máxima sensibilidad y estabilidad
        self.declare_parameter('nthreads', 4)  # 4 subprocesos CPU
        self.declare_parameter('camera_frame', 'camera_0354')
        self.declare_parameter('publish_tf', False)  # Desactivado: solo publicar /usv5/pose
        self.declare_parameter('publish_annotated_image', False)  # Desactivado por defecto para máxima velocidad
        self.declare_parameter('show_window', False)  # Headless sin GUI para alcanzar el FPS completo del stream

        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.tag_family = self.get_parameter('tag_family').value
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.axis_scale = float(self.get_parameter('axis_scale').value)
        self.quad_decimate = float(self.get_parameter('quad_decimate').value)
        self.nthreads = int(self.get_parameter('nthreads').value)
        self.camera_frame = self.get_parameter('camera_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_annotated_image = self.get_parameter('publish_annotated_image').value
        self.show_window = self.get_parameter('show_window').value

        if self.show_window:
            cv2.namedWindow('AprilTag Detections - camera_0354', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('AprilTag Detections - camera_0354', 800, 800)

        # Matriz de calibración predeterminada (cámara 0354) por si aún no llega CameraInfo
        self.fx = 1109.207609712391
        self.fy = 1114.48004663815
        self.cx = 1019.426843076671
        self.cy = 1033.486904196278
        self.calib_width = 2048.0
        self.calib_height = 2048.0
        self.dist_coeffs = np.zeros(5, dtype=np.float64)

        # Inicializar detector de pupil_apriltags
        self.detector = pupil_apriltags.Detector(
            families=self.tag_family,
            nthreads=self.nthreads,
            quad_decimate=self.quad_decimate,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # Broadcaster de TF y Publicador principal de Pose
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.pub_usv5_pose_stamped = self.create_publisher(PoseStamped, '/usv5/pose', 10)

        if self.publish_annotated_image:
            self.pub_img_raw = self.create_publisher(Image, '/camera_0354/apriltag/image_raw', 10)
            self.pub_img_comp = self.create_publisher(CompressedImage, '/camera_0354/apriltag/image_raw/compressed', 10)

        # Suscriptores compatibles con la fuente client_bridge
        self.sub_cam_info = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 10)
        self.sub_image = self.create_subscription(CompressedImage, self.image_topic, self.image_callback, 10)

        self.get_logger().info(f"Detector de AprilTags iniciado en tópico: {self.image_topic} (Familia: {self.tag_family}, Tamaño: {self.tag_size}m)")

    def camera_info_callback(self, msg: CameraInfo):
        if len(msg.k) == 9 and msg.k[0] > 0:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            if msg.width > 0 and msg.height > 0:
                self.calib_width = float(msg.width)
                self.calib_height = float(msg.height)
            if len(msg.d) >= 5:
                self.dist_coeffs = np.array(msg.d[:5], dtype=np.float64)

    def image_callback(self, msg: CompressedImage):
        if not rclpy.ok():
            return

        # Decodificar JPEG
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Escalar parámetros intrínsecos si la resolución de la imagen difiere de la resolución de calibración
        scale_x = float(w) / self.calib_width if self.calib_width > 0 else 1.0
        scale_y = float(h) / self.calib_height if self.calib_height > 0 else 1.0

        fx_eff = self.fx * scale_x
        fy_eff = self.fy * scale_y
        cx_eff = self.cx * scale_x
        cy_eff = self.cy * scale_y

        camera_params = [fx_eff, fy_eff, cx_eff, cy_eff]

        # Detectar AprilTags
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=camera_params,
            tag_size=self.tag_size
        )

        stamp = msg.header.stamp if msg.header.stamp.sec != 0 else self.get_clock().now().to_msg()
        frame_id = msg.header.frame_id if msg.header.frame_id else self.camera_frame

        # Publicar TF del origen 'odom' en el centro de la cámara (0,0,0)
        if self.publish_tf and rclpy.ok():
            tf_odom = TransformStamped()
            tf_odom.header.stamp = stamp
            tf_odom.header.frame_id = frame_id
            tf_odom.child_frame_id = "odom"
            tf_odom.transform.translation.x = 0.0
            tf_odom.transform.translation.y = 0.0
            tf_odom.transform.translation.z = 0.0
            tf_odom.transform.rotation.w = 1.0
            tf_odom.transform.rotation.x = 0.0
            tf_odom.transform.rotation.y = 0.0
            tf_odom.transform.rotation.z = 0.0
            try:
                self.tf_broadcaster.sendTransform(tf_odom)
            except Exception:
                pass

        K_eff = np.array([[fx_eff, 0, cx_eff], [0, fy_eff, cy_eff], [0, 0, 1]], dtype=np.float64)

        # Dibujar centro óptico (0,0) de la cámara en la imagen
        c_x_img, c_y_img = int(cx_eff), int(cy_eff)
        if self.publish_annotated_image or self.show_window:
            cv2.drawMarker(frame, (c_x_img, c_y_img), (255, 255, 0), cv2.MARKER_CROSS, 24, 2)
            cv2.putText(frame, "Origen (0,0)", (c_x_img + 10, c_y_img + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        for det in detections:
            tag_family = str(det.tag_family, 'utf-8') if isinstance(det.tag_family, bytes) else str(det.tag_family)
            tag_id = int(det.tag_id)

            # Estimar Pose
            if det.pose_R is not None and det.pose_t is not None:
                t = det.pose_t.flatten()

                # Publicar TF del AprilTag desde el origen 'odom' con orientación constante y Z=0
                if self.publish_tf and rclpy.ok():
                    tf_msg = TransformStamped()
                    tf_msg.header.stamp = stamp
                    tf_msg.header.frame_id = "odom"
                    tf_msg.child_frame_id = "usv5"
                    tf_msg.transform.translation.x = float(t[0])
                    tf_msg.transform.translation.y = float(t[1])
                    tf_msg.transform.translation.z = 0.0
                    tf_msg.transform.rotation.w = 1.0
                    tf_msg.transform.rotation.x = 0.0
                    tf_msg.transform.rotation.y = 0.0
                    tf_msg.transform.rotation.z = 0.0
                    try:
                        self.tf_broadcaster.sendTransform(tf_msg)
                    except Exception:
                        pass

                # Publicar en /usv5/pose (PoseStamped con timestamp y posición 2D X,Y en metros, Z=0)
                if rclpy.ok():
                    usv5_stamped = PoseStamped()
                    usv5_stamped.header.stamp = stamp
                    usv5_stamped.header.frame_id = "odom"
                    usv5_stamped.pose.position.x = float(t[0])
                    usv5_stamped.pose.position.y = float(t[1])
                    usv5_stamped.pose.position.z = 0.0
                    usv5_stamped.pose.orientation.w = 1.0
                    usv5_stamped.pose.orientation.x = 0.0
                    usv5_stamped.pose.orientation.y = 0.0
                    usv5_stamped.pose.orientation.z = 0.0

                    try:
                        self.pub_usv5_pose_stamped.publish(usv5_stamped)
                    except Exception:
                        pass

                # Dibujar en la imagen sólo posición y etiqueta del Tag
                if self.publish_annotated_image or self.show_window:
                    cx_i, cy_i = int(det.center[0]), int(det.center[1])

                    # Línea desde el origen óptico (0,0) hasta el centro del tag
                    cv2.line(frame, (c_x_img, c_y_img), (cx_i, cy_i), (255, 255, 0), 1, cv2.LINE_AA)

                    # Dibujar polígono del tag
                    corners = np.int32(det.corners)
                    cv2.polylines(frame, [corners], True, (0, 255, 255), 3)

                    # Dibujar centro del tag
                    cv2.circle(frame, (cx_i, cy_i), 6, (0, 0, 255), -1)

                    # Mostrar ID y posición 3D: Plano (X,Y) y Distancia Z en metros desde el origen (0,0)
                    pos_str = f"ID:{det.tag_id} [Plano X:{t[0]:.2f}m Y:{t[1]:.2f}m | Dist Z:{t[2]:.2f}m]"
                    cv2.putText(frame, pos_str, (cx_i - 50, cy_i - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        if not rclpy.ok():
            return

        # Publicar imagen anotada
        if self.publish_annotated_image and rclpy.ok():
            try:
                # Compressed
                ok, encoded_img = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    comp_out = CompressedImage()
                    comp_out.header.stamp = stamp
                    comp_out.header.frame_id = frame_id
                    comp_out.format = 'jpeg'
                    comp_out.data = encoded_img.tobytes()
                    self.pub_img_comp.publish(comp_out)

                # Raw
                h, w, c = frame.shape
                raw_out = Image()
                raw_out.header.stamp = stamp
                raw_out.header.frame_id = frame_id
                raw_out.height = h
                raw_out.width = w
                raw_out.encoding = 'bgr8'
                raw_out.step = w * c
                raw_out.data = frame.tobytes()
                self.pub_img_raw.publish(raw_out)
            except Exception:
                pass

        # Mostrar en pantalla mediante ventana OpenCV si show_window está activo
        if self.show_window:
            cv2.imshow('AprilTag Detections - camera_0354', frame)
            cv2.waitKey(1)

    def destroy_node(self):
        if self.show_window:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
