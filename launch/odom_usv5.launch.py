from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mjpeg_url_arg = DeclareLaunchArgument(
        'mjpeg_url',
        default_value='http://192.168.1.50:8080/stream?topic=/camera_0354/image_raw',
        description='URL del stream MJPEG remoto de la cámara'
    )

    tag_family_arg = DeclareLaunchArgument(
        'tag_family',
        default_value='tag36h11',
        description='Familia del AprilTag'
    )

    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.16',
        description='Tamaño del AprilTag en metros'
    )

    # 1. Bridge de Cámara Remota (HTTP Stream -> ROS 2 Image topics)
    client_bridge_node = Node(
        package='pladypos',
        executable='client_bridge.py',
        name='client_bridge_node',
        output='screen',
        parameters=[{
            'host_ip': '10.250.253.1',
            'camera_topic': '/camera_0354/camera_0354/image_raw',
            'port': 8082
        }]
    )

    # 2. Detector de AprilTags (/camera_0354/image_raw/compressed -> /usv5/pose)
    apriltag_detector_node = Node(
        package='pladypos',
        executable='apriltag_detector.py',
        name='apriltag_detector_node',
        output='screen',
        parameters=[{
            'image_topic': '/camera_0354/image_raw/compressed',
            'tag_family': LaunchConfiguration('tag_family'),
            'tag_size': LaunchConfiguration('tag_size'),
            'publish_tf': False
        }]
    )

    # 3. Nodo de Odometría USV5 (/usv5/pose + IMU -> /usv5/odom a 10 Hz)
    pose_imu_odometry_node = Node(
        package='pladypos',
        executable='pose_imu_odometry.py',
        name='pose_imu_odometry_node',
        output='screen',
        parameters=[{
            'pose_topic': '/usv5/pose',
            'imu_topic': '/imu/data',
            'odom_topic': '/usv5/odom',
            'publish_rate': 10.0,
            'frame_id': 'odom',
            'child_frame_id': 'usv5',
            'publish_tf': True
        }]
    )

    return LaunchDescription([
        mjpeg_url_arg,
        tag_family_arg,
        tag_size_arg,
        client_bridge_node,
        apriltag_detector_node,
        pose_imu_odometry_node
    ])
