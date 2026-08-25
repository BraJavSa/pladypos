#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

def quat_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return w, x, y, z

class FluImuPublisher(Node):
    def __init__(self):
        super().__init__('flu_imu_publisher')

        self.declare_parameter('input_topic', '/usv5/imu/data')
        self.declare_parameter('output_topic', '/usv5/flu_imu/data')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.sub = self.create_subscription(Imu, in_topic, self.imu_cb, 10)
        self.pub = self.create_publisher(Imu, out_topic, 10)

        # Quaternion representing the static rotation from Sensor Frame to Robot Frame (FLU)
        # Robot X = Sensor Y, Robot Y = Sensor X, Robot Z = -Sensor Z
        # This corresponds to a 180 deg rotation around the axis (x=1, y=1, z=0)
        # q = [w=0, x=0.70710678, y=0.70710678, z=0]
        self.q_offset = (0.0, 0.7071067811865476, 0.7071067811865476, 0.0)

        self.get_logger().info(f"FluImuPublisher started: {in_topic} -> {out_topic}")

    def imu_cb(self, msg: Imu):
        out_msg = Imu()
        out_msg.header = msg.header
        # Opcional: renombrar el frame_id si lo deseas
        out_msg.header.frame_id = 'imu_link'

        # 1. Rotar la Orientacion (Cuaternion)
        q_sensor = (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        
        # q_robot = q_sensor * q_offset
        w, x, y, z = quat_mult(q_sensor, self.q_offset)
        
        out_msg.orientation.w = w
        out_msg.orientation.x = x
        out_msg.orientation.y = y
        out_msg.orientation.z = z
        out_msg.orientation_covariance = msg.orientation_covariance

        # 2. Rotar la Velocidad Angular
        # Robot X = Sensor Y
        # Robot Y = Sensor X
        # Robot Z = -Sensor Z
        out_msg.angular_velocity.x = msg.angular_velocity.y
        out_msg.angular_velocity.y = msg.angular_velocity.x
        out_msg.angular_velocity.z = -msg.angular_velocity.z
        out_msg.angular_velocity_covariance = msg.angular_velocity_covariance

        # 3. Rotar la Aceleracion Lineal
        out_msg.linear_acceleration.x = msg.linear_acceleration.y
        out_msg.linear_acceleration.y = msg.linear_acceleration.x
        out_msg.linear_acceleration.z = -msg.linear_acceleration.z
        out_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FluImuPublisher()
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
