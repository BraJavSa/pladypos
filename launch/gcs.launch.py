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
        description='Launch joy_node on the GCS laptop to send /usv5/joy to the server'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 with pool.rviz on the GCS laptop'
    )

    water_depth_arg = DeclareLaunchArgument(
        'water_depth',
        default_value='5.1',
        description='Distance from overhead camera to pool water surface in meters'
    )

    # 1. GCS Joystick Node (publishes /usv5/joy -> rawfie server teleop_mixer -> /usv5/pwm_out)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=ns,
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joy'))
    )

    # 2. Local Static TF (camera -> odom) so RViz always has the full TF tree immediately
    static_tf_camera_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_odom_gcs',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', LaunchConfiguration('water_depth'),
            '--qx', '1.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '0.0',
            '--frame-id', 'camera',
            '--child-frame-id', 'odom'
        ]
    )

    # 3. Local Pool & Figure-8 Reference Marker Publisher (ensures RViz always displays pool + Figure-8)
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

    # 4. RViz2 Monitoring Window (displays pool boundary, Figure-8 reference, USV TF, and executed path)
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
        water_depth_arg,
        joy_node,
        static_tf_camera_to_odom,
        pool_boundary_node,
        rviz_node,
    ])
