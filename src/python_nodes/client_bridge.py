#!/usr/bin/env python3

import os
# Remover ROS_DISCOVERY_SERVER para asegurar el descubrimiento DDS nativo por multicast en la red local
os.environ.pop("ROS_DISCOVERY_SERVER", None)

import sys
import time
import threading
import urllib.request
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo

class RemoteCameraBridgeNode(Node):
    def __init__(self):
        super().__init__('remote_camera_bridge')
        
        self.declare_parameter('host_ip', '10.250.253.1')
        self.declare_parameter('camera_topic', '/camera_0354/camera_0354/image_raw')
        self.declare_parameter('port', 8082)
        
        self.host_ip = str(self.get_parameter('host_ip').value)
        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.port = int(self.get_parameter('port').value)
        
        self.stream_url = f"http://{self.host_ip}:{self.port}/stream?topic={self.camera_topic}&type=mjpeg"
        
        # Publicadores limpios
        self.pub_raw = self.create_publisher(Image, '/camera_0354/image_raw', 10)
        self.pub_comp = self.create_publisher(CompressedImage, '/camera_0354/image_raw/compressed', 10)
        self.pub_info = self.create_publisher(CameraInfo, '/camera_0354/camera_info', 10)
        
        # Suscriptores dummy para mantener activa la presencia en graph discovery
        self.sub_dummy1 = self.create_subscription(CompressedImage, '/camera_0354/image_raw/compressed', lambda msg: None, 10)
        self.sub_dummy2 = self.create_subscription(CameraInfo, '/camera_0354/camera_info', lambda msg: None, 10)
        
        # Cargar metadatos exactos de calibración de lente para la cámara 0354
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = 'camera_0354'
        self.camera_info.height = 2048
        self.camera_info.width = 2048
        self.camera_info.distortion_model = 'plumb_bob'
        self.camera_info.d = [-0.1268312444881338, 0.04624447483590838, -0.0006651708835886797, -0.0003698069893855444, 0.0]
        self.camera_info.k = [1109.207609712391, 0.0, 1019.426843076671, 0.0, 1114.48004663815, 1033.486904196278, 0.0, 0.0, 1.0]
        self.camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info.p = [1032.697143554688, 0.0, 1018.47285762371, 0.0, 0.0, 1035.837158203125, 1030.122685431561, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        self.latest_jpg = None
        self.lock = threading.Lock()
        self.running = True
        
        self.worker_thread = threading.Thread(target=self.http_worker, daemon=True)
        self.worker_thread.start()
        
        # Timer de ROS 2 a 30 Hz sin logs ruidosos
        self.timer = self.create_timer(0.033, self.timer_callback)

    def http_worker(self):
        while self.running and rclpy.ok():
            try:
                stream = urllib.request.urlopen(self.stream_url, timeout=5)
                bytes_data = bytes()
                
                while self.running and rclpy.ok():
                    chunk = stream.read(16384)
                    if not chunk:
                        break
                    bytes_data += chunk
                    
                    a = bytes_data.find(b'\xff\xd8')
                    b = bytes_data.find(b'\xff\xd9')
                    
                    if a != -1 and b != -1:
                        if b > a:
                            jpg = bytes_data[a:b+2]
                            bytes_data = bytes_data[b+2:]
                            with self.lock:
                                self.latest_jpg = jpg
                        else:
                            bytes_data = bytes_data[a:]
            except Exception:
                if self.running:
                    time.sleep(1.0)

    def timer_callback(self):
        if not rclpy.ok():
            return
            
        jpg = None
        with self.lock:
            if self.latest_jpg is not None:
                jpg = self.latest_jpg
                self.latest_jpg = None
                
        if jpg is not None:
            now = self.get_clock().now().to_msg()
            
            try:
                # 1. Publicar CompressedImage (Ultra liviano)
                comp_msg = CompressedImage()
                comp_msg.header.stamp = now
                comp_msg.header.frame_id = 'camera_0354'
                comp_msg.format = 'jpeg'
                comp_msg.data = jpg
                if rclpy.ok():
                    self.pub_comp.publish(comp_msg)
                
                # 2. Publicar CameraInfo (Parámetros e intrínsecos de la lente)
                self.camera_info.header.stamp = now
                if rclpy.ok():
                    self.pub_info.publish(self.camera_info)
                
                # 3. Publicar Image RAW (para visualizar en rqt_image_view o RViz)
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None and rclpy.ok():
                    h, w, c = frame.shape
                    raw_msg = Image()
                    raw_msg.header.stamp = now
                    raw_msg.header.frame_id = 'camera_0354'
                    raw_msg.height = h
                    raw_msg.width = w
                    raw_msg.encoding = 'bgr8'
                    raw_msg.is_bigendian = 0
                    raw_msg.step = w * c
                    raw_msg.data = frame.tobytes()
                    self.pub_raw.publish(raw_msg)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = RemoteCameraBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
