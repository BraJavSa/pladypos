#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
import threading
import glob
import os
import time
from std_msgs.msg import Float32, Float32MultiArray, Int16MultiArray
from sensor_msgs.msg import Joy

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 115200)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        self.spektrum_pub = self.create_publisher(Int16MultiArray, 'spektrum', 10)
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)
        self.telemetry_pub = self.create_publisher(Float32, 'telemetry', 10)

        self.pwm_sub = self.create_subscription(
            Float32MultiArray,
            'pwm_out',
            self.pwm_callback,
            10
        )

        self.ser = None
        self.lock = threading.Lock()
        
        if self.port == 'auto':
            self.get_logger().info("Searching for Arduino bridge port dynamically...")
            detected_port, active_ser = self.find_arduino_port()
            if detected_port:
                self.port = detected_port
                self.ser = active_ser
                self.get_logger().info(f"Dynamically detected and connected to Arduino on: {self.port}")
            else:
                self.get_logger().warn("Could not detect Arduino on startup, will keep scanning...")
        else:
            self.connect_serial()

        self.joy_min = 342
        self.joy_max = 1706

        self.running = True
        self.read_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.read_thread.start()

    def connect_serial(self):
        with self.lock:
            try:
                if self.ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
                self.get_logger().info(f"Connected to Arduino on {self.port} at {self.baud} baud.")
            except Exception as e:
                self.get_logger().warn(f"Could not open serial port {self.port}: {e}")
                self.ser = None

    def recover_serial(self):
        if self.port == 'auto':
            self.get_logger().info("Scanning for Arduino bridge port to reconnect...")
            detected_port, active_ser = self.find_arduino_port()
            if detected_port:
                with self.lock:
                    self.port = detected_port
                    self.ser = active_ser
                self.get_logger().info(f"Reconnected successfully to Arduino on: {self.port}")
        else:
            self.get_logger().info(f"Attempting to reconnect to serial port {self.port}...")
            self.connect_serial()

    def find_arduino_port(self):
        by_id_path = '/dev/serial/by-id'
        by_id_ports = []
        if os.path.exists(by_id_path):
            for f in os.listdir(by_id_path):
                path = os.path.join(by_id_path, f)
                by_id_ports.append(os.path.realpath(path))
                
        candidates = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        candidates = sorted(list(set([os.path.realpath(c) for c in candidates] + by_id_ports)))

        for port in candidates:
            try:
                ser = serial.Serial(port, self.baud, timeout=0.15)
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                ser.write(bytearray([0xFF, 0xFF, 0xF0, 0x00, 0xF0]))
                time.sleep(0.05)
                buf = ser.read(30)
                if b"PLADYPOS_BRIDGE" in buf:
                    ser.timeout = 0.1
                    return port, ser
                ser.close()
            except Exception:
                continue
        return None, None

    def scale_value(self, val):
        return 2.0 * float(val - self.joy_min) / (self.joy_max - self.joy_min) - 1.0

    def pwm_callback(self, msg):
        with self.lock:
            if not self.ser or not self.ser.is_open or not hasattr(msg, 'data'):
                return

            try:
                vals = list(msg.data)
            except TypeError:
                vals = [msg.data]

            payload = bytearray()
            for val in vals[:4]:
                pwm_val = int(1500 + 400 * max(-1.0, min(1.0, val)))
                payload.extend(struct.pack('<H', pwm_val))

            header = bytearray([0xFF, 0xFF, 0x00, len(payload)])
            checksum = sum(header[2:] + payload) & 0xFF
            packet = header + payload + bytearray([checksum])

            try:
                self.ser.write(packet)
            except Exception as e:
                self.get_logger().warn(f"Failed to send PWM to Arduino: {e}")
                # Trigger recovery in a non-blocking thread
                threading.Thread(target=self.recover_serial, daemon=True).start()

    def receive_loop(self):
        state = 0
        payload_type = 0
        payload_len = 0
        payload = bytearray()

        while rclpy.ok() and self.running:
            try:
                with self.lock:
                    if not self.ser or not self.ser.is_open:
                        ser_ok = False
                    else:
                        ser_ok = True
                
                if not ser_ok:
                    self.recover_serial()
                    time.sleep(2.0)
                    continue

                b = b''
                with self.lock:
                    if self.ser and self.ser.is_open:
                        b = self.ser.read(1)
                
                if not b:
                    continue
                val = b[0]

                if state == 0:
                    if val == 0xFF:
                        state = 1
                elif state == 1:
                    if val == 0xFF:
                        state = 2
                    else:
                        state = 0
                elif state == 2:
                    payload_type = val
                    state = 3
                elif state == 3:
                    payload_len = val
                    payload = bytearray()
                    state = 4
                elif state == 4:
                    payload.append(val)
                    if len(payload) == payload_len:
                        state = 5
                elif state == 5:
                    checksum = (payload_type + payload_len + sum(payload)) & 0xFF
                    if checksum == val:
                        self.process_packet(payload_type, payload)
                    state = 0

            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                with self.lock:
                    if self.ser:
                        try:
                            self.ser.close()
                        except Exception:
                            pass
                        self.ser = None
                time.sleep(2.0)

    def process_packet(self, p_type, payload):
        if p_type == 1:
            if len(payload) == 12:
                channels = list(struct.unpack('<6H', payload))
                
                sp_msg = Int16MultiArray()
                sp_msg.data = channels
                self.spektrum_pub.publish(sp_msg)

                joy_msg = Joy()
                joy_msg.header.stamp = self.get_clock().now().to_msg()
                joy_msg.header.frame_id = 'base_link'
                joy_msg.axes = [
                    self.scale_value(channels[1]),
                    self.scale_value(channels[0]),
                    self.scale_value(channels[2]),
                    0.0, 0.0, 0.0
                ]
                self.joy_pub.publish(joy_msg)

        elif p_type == 2:
            if len(payload) == 4:
                voltage = struct.unpack('<f', payload)[0]
                tel_msg = Float32()
                tel_msg.data = voltage
                self.telemetry_pub.publish(tel_msg)

    def destroy_node(self):
        self.running = False
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
