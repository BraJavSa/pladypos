#!/usr/bin/env python3
import os
import yaml
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
        
        # Load motor mapping config with default fallback
        self.motor_map = {
            1: "Front Right (FR)",
            2: "Front Left (FL)",
            3: "Back Right (BR)",
            4: "Back Left (BL)"
        }
        
        config_path = '/home/brayan/ros2_ws/src/pladypos/config/motor_mapping.yaml'
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    config_data = yaml.safe_load(f)
                    if config_data and "motor_mapping" in config_data:
                        self.motor_map = config_data["motor_mapping"]
                        self.get_logger().info(f"Loaded motor mapping from {config_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load motor mapping: {e}. Using default.")
        else:
            self.get_logger().info("No motor mapping file found. Using default mapping.")

    def joy_callback(self, msg):
        if len(msg.axes) < 3:
            return

        sway = msg.axes[0]
        surge = msg.axes[1]
        yaw = msg.axes[2]

        # Calculate standard thrust values for each physical position
        u_FR = surge - sway - yaw
        u_FL = surge + sway + yaw
        u_BR = -surge - sway + yaw
        u_BL = -surge + sway - yaw

        # Max normalize to prevent thruster saturation
        max_val = max(abs(u_FR), abs(u_FL), abs(u_BR), abs(u_BL))
        if max_val > 1.0:
            u_FR /= max_val
            u_FL /= max_val
            u_BR /= max_val
            u_BL /= max_val

        # Map to the correct channels
        data = [0.0, 0.0, 0.0, 0.0]
        for channel, position in self.motor_map.items():
            ch_idx = int(channel) - 1
            if 0 <= ch_idx < 4:
                if "Front Right" in position or "FR" in position:
                    data[ch_idx] = float(u_FR)
                elif "Front Left" in position or "FL" in position:
                    data[ch_idx] = float(u_FL)
                elif "Back Right" in position or "BR" in position:
                    data[ch_idx] = float(u_BR)
                elif "Back Left" in position or "BL" in position:
                    data[ch_idx] = float(u_BL)

        pwm_msg = Float32MultiArray()
        pwm_msg.data = data
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
