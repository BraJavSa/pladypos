#!/usr/bin/env python3
"""
Transparent ROS 2 Topic TCP Bridge.
Relays topics bidirectionally between the USV (boat) and GCS (laptop) over a single reliable TCP socket.
"""
import sys
import time
import socket
import threading
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
from sensor_msgs.msg import Imu, Joy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class NetworkBridge(Node):
    def __init__(self):
        super().__init__('network_bridge')
        self.declare_parameter('mode', 'server')  # 'server' on boat, 'client' on laptop
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 9876)
        self.declare_parameter('usv_id', 5)

        self.mode = self.get_parameter('mode').value
        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.usv_id = int(self.get_parameter('usv_id').value)
        self.ns = f"usv{self.usv_id}"

        self.conn = None
        self.lock = threading.Lock()
        self.running = True

        # Topic registry: topic_id -> (Topic Name, ROS Message Class)
        self.topic_map = {
            1: (f"/{self.ns}/imu/data", Imu),
            2: (f"/{self.ns}/joy", Joy),
            3: (f"/{self.ns}/odom", Odometry),
            4: (f"/{self.ns}/cmd_vel", Twist)
        }
        self.name_to_id = {name: tid for tid, (name, _) in self.topic_map.items()}

        self.pubs = {}
        self.subs = {}

        if self.mode == 'server':
            # Running on Boat:
            # - Subscribes to local IMU (/usv5/imu/data) and sends over TCP to laptop
            # - Publishes /usv5/joy received from laptop
            self.subs[1] = self.create_subscription(Imu, f"/{self.ns}/imu/data", lambda msg: self.send_ros_msg(1, msg), 10)
            self.pubs[2] = self.create_publisher(Joy, f"/{self.ns}/joy", 10)
            threading.Thread(target=self._run_server, daemon=True).start()
        else:
            # Running on Laptop (client):
            # - Subscribes to local Joy (/usv5/joy) and sends over TCP to boat
            # - Publishes /usv5/imu/data received from boat
            self.pubs[1] = self.create_publisher(Imu, f"/{self.ns}/imu/data", 10)
            self.subs[2] = self.create_subscription(Joy, f"/{self.ns}/joy", lambda msg: self.send_ros_msg(2, msg), 10)
            threading.Thread(target=self._run_client, daemon=True).start()

        self.get_logger().info(f"NetworkBridge initialized in {self.mode.upper()} mode on {self.host}:{self.port}")

    def send_ros_msg(self, topic_id: int, msg):
        raw = serialize_message(msg)
        length = len(raw)
        packet = bytearray([0xAA, 0x55, topic_id]) + length.to_bytes(4, 'big') + raw
        with self.lock:
            if self.conn:
                try:
                    self.conn.sendall(packet)
                except Exception:
                    self.conn = None

    def _handle_stream(self, sock):
        self.conn = sock
        self.get_logger().info("TCP Bridge connected!")
        buf = bytearray()
        while self.running:
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buf.extend(chunk)

                while len(buf) >= 7:
                    if buf[0] != 0xAA or buf[1] != 0x55:
                        buf.pop(0)
                        continue
                    topic_id = buf[2]
                    msg_len = int.from_bytes(buf[3:7], 'big')
                    total = 7 + msg_len
                    if len(buf) < total:
                        break
                    raw_data = bytes(buf[7:total])
                    buf = buf[total:]

                    if topic_id in self.pubs and topic_id in self.topic_map:
                        _, msg_cls = self.topic_map[topic_id]
                        msg = deserialize_message(raw_data, msg_cls)
                        self.pubs[topic_id].publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Socket read error: {e}")
                break
        with self.lock:
            self.conn = None
        self.get_logger().info("TCP Bridge disconnected.")

    def _run_server(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        while self.running:
            sock, addr = srv.accept()
            self.get_logger().info(f"Accepted connection from {addr}")
            self._handle_stream(sock)

    def _run_client(self):
        while self.running:
            if not self.conn:
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.connect((self.host, self.port))
                    self._handle_stream(sock)
                except Exception:
                    time.sleep(1.0)
            else:
                time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = NetworkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
