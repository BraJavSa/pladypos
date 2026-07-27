import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
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
        src_path = '/home/brayan/ros2_ws/src/pladypos/config/usv_config.yaml'
        if os.path.exists(src_path):
            try:
                with open(src_path, 'r') as f:
                    config_data = yaml.safe_load(f)
                    usv_id = config_data.get('usv_id', 5)
            except Exception:
                pass

    ns = f"usv{usv_id}"

    # Declare arguments
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate for the IMU and Arduino communication'
    )
    
    port_imu_arg = DeclareLaunchArgument(
        'port_imu',
        default_value='auto',
        description='Port of the IMU'
    )

    port_arduino_arg = DeclareLaunchArgument(
        'port_arduino',
        default_value='auto',
        description='Port of the Arduino'
    )

    # Core nodes
    imu_driver_node = Node(
        package='pladypos',
        executable='imu_driver.py',
        name='imu_driver_node',
        namespace=ns,
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port_imu'),
            'baud': LaunchConfiguration('baud'),
            'use_ned': True
        }]
    )

    serial_bridge_node = Node(
        package='pladypos',
        executable='serial_bridge.py',
        name='serial_bridge_node',
        namespace=ns,
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port_arduino'),
            'baud': LaunchConfiguration('baud')
        }]
    )

    teleop_mixer_node = Node(
        package='pladypos',
        executable='teleop_mixer.py',
        name='teleop_mixer_node',
        namespace=ns,
        output='screen'
    )

    complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node',
        namespace=ns,
        output='screen',
        remappings=[
            ('imu/data_raw', 'imu/data_raw'),
            ('imu/mag', 'imu/mag')
        ]
    )

    # Launch Kinect v2 Bridge Node directly in 2D / Webcam mode (No PointCloud container)
    kinect_webcam_node = Node(
        package='kinect2_bridge',
        executable='kinect2_bridge',
        name='kinect2_bridge',
        emulate_tty=True,
        parameters=[{
            'base_name': 'kinect2',
            'sensor': '',
            'publish_tf': False,        # Disable transform calculations
            'worker_threads': 2,        # Restrict threads to save CPU
            'fps_limit': -1.0,          # Correct double type
            'use_png': False,
            'jpeg_quality': 85,         # Compressed JPEG stream for WiFi efficiency
            'png_level': 1,
            'depth_method': 'default',
            'reg_method': 'default'
        }],
        remappings=[
            ('/kinect2/qhd/image_color_rect', f'/{ns}/camera/image_raw'),
            ('/kinect2/qhd/image_color_rect/compressed', f'/{ns}/camera/image_raw/compressed'),
            ('/kinect2/sd/image_depth_rect', f'/{ns}/camera/depth_raw'),
            ('/kinect2/sd/image_depth_rect/compressed', f'/{ns}/camera/depth_raw/compressed'),
        ],
        output='screen'
    )

    depth_to_mono8_node = Node(
        package='pladypos',
        executable='depth_to_mono8.py',
        name='depth_to_mono8_node',
        output='screen',
        parameters=[{
            'input_topic': f'/{ns}/camera/depth_raw',
            'output_topic': f'/{ns}/camera/depth',
            'max_depth': 12.0
        }]
    )

    filter_start_event = RegisterEventHandler(
        OnProcessStart(
            target_action=imu_driver_node,
            on_start=[complementary_filter_node]
        )
    )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': 8080,
            'address': '0.0.0.0',
            'type': 'mjpeg',
            'default_transport': 'raw'
        }],
        output='screen'
    )

    return LaunchDescription([
        baud_arg,
        port_imu_arg,
        port_arduino_arg,
        imu_driver_node,
        serial_bridge_node,
        teleop_mixer_node,
        kinect_webcam_node,
        depth_to_mono8_node,
        web_video_server_node,
        filter_start_event
    ])
