import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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
    rviz_config = os.path.join(pkg_share, 'config', 'pool.rviz')

    use_joy_arg = DeclareLaunchArgument(
        'use_joy',
        default_value='true',
        description='Launch joy_node on the GCS laptop to send /usv5/joy to the boat'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 with pool.rviz on the GCS laptop'
    )

    # 1. GCS Joystick Node (publishes /usv5/joy -> rawfie Zenoh Router -> USV teleop_mixer)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=ns,
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joy'))
    )

    # 2. Local Pool & Figure-8 Reference Marker Publisher (ensures RViz always displays pool + Figure-8)
    pool_boundary_node = Node(
        package='pladypos',
        executable='pool_boundary_publisher.py',
        name='pool_boundary_publisher_gcs',
        output='screen',
        parameters=[{
            'frame_id': 'odom',
            'marker_topic': '/pool_boundary'
        }]
    )

    # 3. RViz2 Monitoring Window (displays pool boundary, Figure-8 reference, USV TF, and executed path)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_joy_arg,
        use_rviz_arg,
        joy_node,
        pool_boundary_node,
        rviz_node,
    ])
