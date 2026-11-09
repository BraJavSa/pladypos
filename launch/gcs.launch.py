import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    GCS Visualization Launch File (Laptop, ROS_DOMAIN_ID=0):
    Launches ONLY RViz2, static TF (camera -> odom), and pool boundary markers.
    Does NOT launch joystick/teleop so manual control never overwrites autonomous control.
    Use `ros2 launch pladypos teleop.launch.py` in a separate terminal when manual joystick
    positioning is needed.
    """
    pkg_share = get_package_share_directory('pladypos')
    rviz_config = os.path.join(pkg_share, 'config', 'pool.rviz')

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

    # 1. Local Static TF (camera -> odom) so RViz always has the full TF tree immediately
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
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        use_rviz_arg,
        water_depth_arg,
        static_tf_camera_to_odom,
        pool_boundary_node,
        rviz_node,
    ])
