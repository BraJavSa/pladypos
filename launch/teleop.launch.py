import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """
    Standalone Manual Teleoperation Launch File (Laptop, ROS_DOMAIN_ID=0):
    Launches joy_node + teleop_mixer_node to manually position the USV with the joystick.
    Stop this launch file (Ctrl+C) once the boat is positioned before launching the
    autonomous controller on the USV (`ros2 launch pladypos inverse_dynamics_8.launch.py`).
    """
    pkg_share = get_package_share_directory('pladypos')
    usv_id = 5
    try:
        config_path = os.path.join(pkg_share, 'config', 'usv_config.yaml')
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                usv_id = config_data.get('usv_id', 5)
    except Exception:
        pass

    ns = f"usv{usv_id}"

    # 1. Joystick Driver (/usv5/joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=ns,
        output='screen'
    )

    # 2. Teleoperation Mixer (maps /usv5/joy -> /usv5/pwm_out; sends [0,0,0,0] on Ctrl+C)
    teleop_mixer_node = Node(
        package='pladypos',
        executable='teleop_mixer.py',
        name='teleop_mixer_node',
        namespace=ns,
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        joy_node,
        teleop_mixer_node,
    ])

