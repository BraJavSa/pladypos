from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    host_ip_arg = DeclareLaunchArgument(
        'host_ip',
        default_value='10.250.253.1',
        description='IP del host emisor del stream MJPEG'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera_0354/camera_0354/image_raw',
        description='Tópico original de la cámara remota'
    )

    client_bridge_node = Node(
        package='pladypos',
        executable='client_bridge.py',
        name='client_bridge_node',
        output='screen',
        arguments=[
            LaunchConfiguration('host_ip'),
            LaunchConfiguration('camera_topic')
        ]
    )

    return LaunchDescription([
        host_ip_arg,
        camera_topic_arg,
        client_bridge_node
    ])
