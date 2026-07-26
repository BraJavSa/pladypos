#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class TeleopMixer(Node):
    def __init__(self):
        super().__init__('teleop_mixer')

        self.pwm_pub = self.create_publisher(Float32MultiArray, 'pwm_out', 10)
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        if len(msg.axes) < 3:
            return

        sway = msg.axes[0]
        surge = msg.axes[1]
        yaw = msg.axes[2]

        u1 = surge - sway - yaw
        u2 = surge + sway + yaw
        u3 = -surge - sway + yaw
        u4 = -surge + sway - yaw

        max_val = max(abs(u1), abs(u2), abs(u3), abs(u4))
        if max_val > 1.0:
            u1 /= max_val
            u2 /= max_val
            u3 /= max_val
            u4 /= max_val

        pwm_msg = Float32MultiArray()
        pwm_msg.data = [float(u1), float(u2), float(u3), float(u4)]
        self.pwm_pub.publish(pwm_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopMixer()
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
