import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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
        pass

    ns = f"usv{usv_id}"

    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='0354',
        description='Camera identifier (e.g. 0354 or 0352)'
    )

    use_joy_arg = DeclareLaunchArgument(
        'use_joy',
        default_value='true',
        description='Launch joy_node locally on the GCS to send joystick commands'
    )

    tag_id_arg = DeclareLaunchArgument(
        'tag_id',
        default_value='285',
        description='Target AprilTag ID'
    )

    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.25',
        description='AprilTag side length in meters'
    )

    water_depth_arg = DeclareLaunchArgument(
        'water_depth',
        default_value='5.1',
        description='Distance from overhead camera to pool water surface in meters'
    )

    max_yaw_diff_arg = DeclareLaunchArgument(
        'max_yaw_diff_deg',
        default_value='45.0',
        description='Maximum allowed yaw discrepancy between visual pose and IMU'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Broadcast TF transforms (camera -> odom static, and odom -> usv5 dynamic)'
    )

    stream_url_expr = PythonExpression([
        "'http://10.250.253.1:8083/stream?topic=/camera_' + '",
        LaunchConfiguration('camera'),
        "' + '/camera_' + '",
        LaunchConfiguration('camera'),
        "' + '/image_raw'"
    ])

    calib_url_expr = PythonExpression([
        "'http://10.250.253.1:8083/calibration?camera=' + '",
        LaunchConfiguration('camera'),
        "'"
    ])

    # 1. GCS Joystick Node (sends joystick input to USV teleop mixer)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=ns,
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joy'))
    )

    # 2. Static TF: camera (overhead) -> odom (pool surface center, Z points UP)
    # Origin is at image center at water surface (Z = 5.1m down from camera).
    # Rotated 180 deg around X (qx=1.0, qw=0.0) so that +Z points UP towards ceiling, matching IMU and ENU.
    static_tf_camera_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_odom',
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
        ],
        condition=IfCondition(LaunchConfiguration('publish_tf'))
    )

    # 3. Visual AprilTag Estimator Node (Node 2: camera -> visual position and orientation)
    visual_tag_node = Node(
        package='pladypos',
        executable='visual_tag_estimator.py',
        name='visual_tag_estimator',
        output='screen',
        parameters=[{
            'stream_url': stream_url_expr,
            'calib_url': calib_url_expr,
            'tag_id': LaunchConfiguration('tag_id'),
            'tag_size': LaunchConfiguration('tag_size'),
            'odom_topic': f'/{ns}/visual_odom',
            'tag_detected_topic': f'/{ns}/tag_detected',
            'publish_tf': False,
        }]
    )

    # 4. Direct USV Fusion Node (Deterministic 30 Hz fusion: perfect IMU Roll/Pitch, camera-corrected IMU Yaw, camera X/Y, Z=0)
    # Publishes /usv5/odometry/filtered and broadcasts TF: odom -> usv5_odom (and odom -> usv5)
    direct_fusion_node = Node(
        package='pladypos',
        executable='direct_usv_fusion.py',
        name='direct_usv_fusion',
        output='screen',
        parameters=[{
            'visual_odom_topic': f'/{ns}/visual_odom',
            'imu_topic': f'/{ns}/imu/data',
            'odom_topic': f'/{ns}/odometry/filtered',
            'frame_id': 'odom',
            'child_frame_id': f'{ns}_odom',
            'water_depth': LaunchConfiguration('water_depth'),
            'alpha_pos': 0.35,
            'yaw_corr_rate': 0.05
        }]
    )

    # 5. Pool Boundary Visualizer Node (Publishes pool perimeter Marker in 'odom' frame for RViz)
    pool_boundary_node = Node(
        package='pladypos',
        executable='pool_boundary_publisher.py',
        name='pool_boundary_publisher',
        output='screen',
        parameters=[{
            'frame_id': 'odom',
            'marker_topic': '/pool_boundary'
        }]
    )

    return LaunchDescription([
        camera_arg,
        use_joy_arg,
        tag_id_arg,
        tag_size_arg,
        water_depth_arg,
        max_yaw_diff_arg,
        publish_tf_arg,
        joy_node,
        static_tf_camera_to_odom,
        visual_tag_node,
        direct_fusion_node,
        pool_boundary_node,
    ])
