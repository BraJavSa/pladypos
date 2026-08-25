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

        # ROTACIÓN MATEMÁTICA EXACTA PEDIDA:
        # 1. Rotar 180 grados respecto a Y
        # 2. Rotar 90 grados respecto a Z
        # En el mismo marco (Extrínseco)
        
        # Cuaternión Y 180: w=0, x=0, y=1, z=0
        q_y = (0.0, 0.0, 1.0, 0.0)
        
        # Cuaternión Z 90: w=cos(45)=0.7071, x=0, y=0, z=sin(45)=0.7071
        q_z = (0.7071067811865476, 0.0, 0.0, 0.7071067811865476)
        
        # Rotación extrínseca Z(90) * Y(180) -> Multiplicación de cuaterniones q_z * q_y
        self.q_offset = quat_mult(q_z, q_y) 
        # El resultado es w=0.0, x=-0.70710678, y=0.70710678, z=0.0
        
        # Matriz de rotación final (derivada de Z90 * Y180):
        # R = [[0, -1, 0], 
        #      [-1, 0, 0], 
        #      [0, 0, -1]]
        self.R = [
            [ 0.0, -1.0,  0.0],
            [-1.0,  0.0,  0.0],
            [ 0.0,  0.0, -1.0]
        ]

        self.get_logger().info(f"FluImuPublisher started: {in_topic} -> {out_topic}")
        self.get_logger().info(f"Aplicando Rotacion Fija: 180° en Y seguido de 90° en Z (mismo marco)")

    def imu_cb(self, msg: Imu):
        out_msg = Imu()
        out_msg.header = msg.header
        out_msg.header.frame_id = 'imu_link'

        # 1. Rotar la Orientacion (Cuaternion)
        q_sensor = (msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
        w, x, y, z = quat_mult(q_sensor, self.q_offset)
        
        out_msg.orientation.w = w
        out_msg.orientation.x = x
        out_msg.orientation.y = y
        out_msg.orientation.z = z
        out_msg.orientation_covariance = msg.orientation_covariance

        # 2. Rotar la Velocidad Angular usando la matriz R
        vx, vy, vz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        out_msg.angular_velocity.x = self.R[0][0]*vx + self.R[0][1]*vy + self.R[0][2]*vz
        out_msg.angular_velocity.y = self.R[1][0]*vx + self.R[1][1]*vy + self.R[1][2]*vz
        out_msg.angular_velocity.z = self.R[2][0]*vx + self.R[2][1]*vy + self.R[2][2]*vz
        out_msg.angular_velocity_covariance = msg.angular_velocity_covariance

        # 3. Rotar la Aceleracion Lineal usando la matriz R
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        out_msg.linear_acceleration.x = self.R[0][0]*ax + self.R[0][1]*ay + self.R[0][2]*az
        out_msg.linear_acceleration.y = self.R[1][0]*ax + self.R[1][1]*ay + self.R[1][2]*az
        out_msg.linear_acceleration.z = self.R[2][0]*ax + self.R[2][1]*ay + self.R[2][2]*az
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
