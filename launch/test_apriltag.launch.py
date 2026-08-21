from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declarar argumentos configurables para la prueba de AprilTag
    stream_url_arg = DeclareLaunchArgument(
        'stream_url',
        default_value='http://10.250.253.1:8082/stream?topic=/camera_0354/camera_0354/image_raw&type=ros_compressed',
        description='URL del stream MJPEG directo desde web_video_server.'
    )

    tag_family_arg = DeclareLaunchArgument(
        'tag_family',
        default_value='tag36h11',
        description='Familia del AprilTag (ej. tag36h11)'
    )

    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.16',
        description='Tamaño del AprilTag en metros'
    )

    quad_decimate_arg = DeclareLaunchArgument(
        'quad_decimate',
        default_value='1.0',
        description='Decimación interna (1.0 = resolución completa 2048x2048 para máxima precisión)'
    )

    enable_clahe_arg = DeclareLaunchArgument(
        'enable_clahe',
        default_value='True',
        description='Activa ecualización adaptativa de contraste (CLAHE) adicional'
    )

    clahe_clip_limit_arg = DeclareLaunchArgument(
        'clahe_clip_limit',
        default_value='3.5',
        description='Límite de contraste para el filtro CLAHE'
    )

    quad_sigma_arg = DeclareLaunchArgument(
        'quad_sigma',
        default_value='0.8',
        description='Suavizado gaussiano previo para mitigar ruido de compresión JPEG'
    )

    contrast_alpha_arg = DeclareLaunchArgument(
        'contrast_alpha',
        default_value='1.3',
        description='Multiplicador de contraste (1.0 = normal, >1.0 = más contraste)'
    )

    brightness_beta_arg = DeclareLaunchArgument(
        'brightness_beta',
        default_value='35.0',
        description='Incremento de brillo directo en niveles de intensidad (0-255)'
    )

    publish_annotated_image_arg = DeclareLaunchArgument(
        'publish_annotated_image',
        default_value='False',
        description='Publica la imagen anotada con el AprilTag detectado en /camera_0354/apriltag/image_raw'
    )

    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='False',
        description='Muestra una ventana GUI local OpenCV con la detección'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='True',
        description='Publica la transformación TF odom -> usv5'
    )

    # Detector de AprilTags Autónomo con Conexión Directa al Stream HTTP MJPEG
    apriltag_detector_node = Node(
        package='pladypos',
        executable='apriltag_detector.py',
        name='apriltag_detector_node',
        output='screen',
        parameters=[{
            'stream_url': LaunchConfiguration('stream_url'),
            'image_topic': '/camera_0354/image_raw/compressed',
            'camera_info_topic': '/camera_0354/camera_info',
            'tag_family': LaunchConfiguration('tag_family'),
            'tag_size': LaunchConfiguration('tag_size'),
            'quad_decimate': LaunchConfiguration('quad_decimate'),
            'enable_clahe': LaunchConfiguration('enable_clahe'),
            'clahe_clip_limit': LaunchConfiguration('clahe_clip_limit'),
            'quad_sigma': LaunchConfiguration('quad_sigma'),
            'contrast_alpha': LaunchConfiguration('contrast_alpha'),
            'brightness_beta': LaunchConfiguration('brightness_beta'),
            'publish_annotated_image': LaunchConfiguration('publish_annotated_image'),
            'show_window': LaunchConfiguration('show_window'),
            'publish_tf': LaunchConfiguration('publish_tf')
        }]
    )

    return LaunchDescription([
        stream_url_arg,
        tag_family_arg,
        tag_size_arg,
        quad_decimate_arg,
        enable_clahe_arg,
        clahe_clip_limit_arg,
        quad_sigma_arg,
        contrast_alpha_arg,
        brightness_beta_arg,
        publish_annotated_image_arg,
        show_window_arg,
        publish_tf_arg,
        apriltag_detector_node
    ])
