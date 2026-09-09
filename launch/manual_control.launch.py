import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    usv_id = 5
    try:
        config_path = os.path.join(get_package_share_directory('pladypos'), 'config', 'usv_config.yaml')
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                usv_id = config_data.get('usv_id', 5)
    except Exception:
        src_path = '/home/brayan/ros2_ws/src/pladypos/config/usv_config.yaml'
        if os.path.exists(src_path):
            try:
                with open(src_path, 'r') as f:
                    config_data = yaml.safe_load(f)
                    usv_id = config_data.get('usv_id', 5)
            except Exception:
                pass

    ns = f"usv{usv_id}"

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=ns,
        output='screen'
    )

    return LaunchDescription([
        joy_node
    ])
