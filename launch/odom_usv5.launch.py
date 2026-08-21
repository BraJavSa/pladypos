from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='0354',
        description='ID de la cámara (ej. 0354, 9835, 0352, 0353)'
    )

    host_arg = DeclareLaunchArgument(
        'host',
        default_value='10.250.253.1',
        description='IP del servidor / Stream Guard Proxy'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8083',
        description='Puerto del Stream Guard Proxy'
    )

    stream_viewer_node = Node(
        package='pladypos',
        executable='stream_viewer.py',
        name='stream_viewer_node',
        output='screen',
        arguments=[
            '--camera', LaunchConfiguration('camera'),
            '--host', LaunchConfiguration('host'),
            '--port', LaunchConfiguration('port')
        ]
    )

    # 2. Nodo de Odometría USV5 (/usv5/pose + IMU -> /usv5/odom a 10 Hz)
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
        camera_arg,
        host_arg,
        port_arg,
        stream_viewer_node,
        pose_imu_odometry_node
    ])
