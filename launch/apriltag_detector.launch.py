from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera_0354/image_raw/compressed',
        description='Tópico comprimido de la cámara'
    )

    tag_family_arg = DeclareLaunchArgument(
        'tag_family',
        default_value='tag36h11',
        description='Familia del AprilTag (ej. tag36h11, tag25h9)'
    )

    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.16',
        description='Tamaño del AprilTag en metros'
    )

    apriltag_detector_node = Node(
        package='pladypos',
        executable='apriltag_detector.py',
        name='apriltag_detector_node',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'tag_family': LaunchConfiguration('tag_family'),
            'tag_size': LaunchConfiguration('tag_size')
        }]
    )

    return LaunchDescription([
        image_topic_arg,
        tag_family_arg,
        tag_size_arg,
        apriltag_detector_node
    ])
