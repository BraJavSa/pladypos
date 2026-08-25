#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory

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
        
        # Declare ROS 2 parameters for deadzone and axis inversions
        self.declare_parameter('yaw_deadzone', 0.15)
        self.declare_parameter('invert_yaw', True)
        self.declare_parameter('invert_sway', True)
        self.declare_parameter('invert_surge', False)

        # Load motor mapping config with default fallback
        self.motor_map = {
            1: "Front Right (FR)",
            2: "Front Left (FL)",
            3: "Back Right (BR)",
            4: "Back Left (BL)"
        }
        
        try:
            package_share_dir = get_package_share_directory('pladypos')
            config_path = os.path.join(package_share_dir, 'config', 'motor_mapping.yaml')
        except Exception:
            config_path = None

        if config_path and os.path.exists(config_path):
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

    def apply_deadzone(self, value: float, deadzone: float) -> float:
        if abs(value) < deadzone:
            return 0.0
        sign = 1.0 if value > 0.0 else -1.0
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)

    def joy_callback(self, msg):
        if len(msg.axes) < 3:
            return

        yaw_deadzone = self.get_parameter('yaw_deadzone').get_parameter_value().double_value
        invert_yaw = self.get_parameter('invert_yaw').get_parameter_value().bool_value
        invert_sway = self.get_parameter('invert_sway').get_parameter_value().bool_value
        invert_surge = self.get_parameter('invert_surge').get_parameter_value().bool_value

        sway_raw = msg.axes[0]
        surge_raw = msg.axes[1]
        yaw_raw = msg.axes[2]

        # Apply rotational deadzone
        yaw = self.apply_deadzone(yaw_raw, yaw_deadzone)
        sway = sway_raw
        surge = surge_raw

        # Invert directions based on configuration
        if invert_sway:
            sway = -sway
        if invert_surge:
            surge = -surge
        if invert_yaw:
            yaw = -yaw

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
