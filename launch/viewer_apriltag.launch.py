from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    stream_url_arg = DeclareLaunchArgument(
        'stream_url',
        default_value='http://10.250.253.1:8082/stream?topic=/camera_0354/camera_0354/image_raw&type=ros_compressed',
        description='URL del stream MJPEG HTTP directo desde web_video_server.'
    )

    tag_family_arg = DeclareLaunchArgument(
        'tag_family',
        default_value='tag16h5',
        description='Familia del AprilTag (ej. tag16h5)'
    )

    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.25',
        description='Tamaño del AprilTag en metros (0.25m = 25cm)'
    )

    target_tag_id_arg = DeclareLaunchArgument(
        'target_tag_id',
        default_value='8',
        description='ID específico a buscar y mostrar (8)'
    )

    min_decision_margin_arg = DeclareLaunchArgument(
        'min_decision_margin',
        default_value='30.0',
        description='Margen mínimo de decisión para rechazar falsos positivos en tag16h5'
    )

    max_hamming_arg = DeclareLaunchArgument(
        'max_hamming',
        default_value='0',
        description='Distancia Hamming máxima permitida (0 = coincidencia de bits 100% perfecta sin error)'
    )

    quad_decimate_arg = DeclareLaunchArgument(
        'quad_decimate',
        default_value='1.0',
        description='Decimación interna (1.0 = máxima resolución)'
    )

    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='True',
        description='Muestra una ventana GUI local OpenCV con la detección y el nombre del tag'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='False',
        description='Publica la transformación TF odom -> usv5'
    )

    apriltag_viewer_node = Node(
        package='pladypos',
        executable='apriltag_viewer.py',
        name='apriltag_viewer_node',
        output='screen',
        parameters=[{
            'stream_url': LaunchConfiguration('stream_url'),
            'image_topic': '/camera_0354/image_raw/compressed',
            'raw_image_topic': '/camera_0354/camera_0354/image_raw',
            'camera_info_topic': '/camera_0354/camera_info',
            'tag_family': LaunchConfiguration('tag_family'),
            'tag_size': LaunchConfiguration('tag_size'),
            'target_tag_id': LaunchConfiguration('target_tag_id'),
            'min_decision_margin': LaunchConfiguration('min_decision_margin'),
            'max_hamming': LaunchConfiguration('max_hamming'),
            'quad_decimate': LaunchConfiguration('quad_decimate'),
            'show_window': LaunchConfiguration('show_window'),
            'publish_tf': LaunchConfiguration('publish_tf')
        }]
    )

    return LaunchDescription([
        stream_url_arg,
        tag_family_arg,
        tag_size_arg,
        target_tag_id_arg,
        min_decision_margin_arg,
        max_hamming_arg,
        quad_decimate_arg,
        show_window_arg,
        publish_tf_arg,
        apriltag_viewer_node
    ])
