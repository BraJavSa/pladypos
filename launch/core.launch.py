import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
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
        pass

    ns = f"usv{usv_id}"

    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate for serial devices (IMU and Arduino Micro)'
    )

    port_imu_arg = DeclareLaunchArgument(
        'port_imu',
        default_value='auto',
        description='Serial port of the SparkFun Razor IMU M0 (or "auto" for dynamic detection)'
    )

    port_arduino_arg = DeclareLaunchArgument(
        'port_arduino',
        default_value='auto',
        description='Serial port of the Arduino Micro (or "auto" for dynamic detection)'
    )

    use_joy_arg = DeclareLaunchArgument(
        'use_joy',
        default_value='false',
        description='Launch joy_node locally on the USV (set to true if joystick is connected directly to the USV)'
    )

    # 1. Serial Bridge to Arduino Micro (Spektrum RC Radio, motor PWM output, and feedback)
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

    # 2. Teleoperation Mixer (Maps joystick input to thruster allocation PWM)
    teleop_mixer_node = Node(
        package='pladypos',
        executable='teleop_mixer.py',
        name='teleop_mixer_node',
        namespace=ns,
        output='screen'
    )

    # 3. Optional local joystick node (if connected directly to USV)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=ns,
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_joy'))
    )

    # 4. IMU Driver for SparkFun 9DoF Razor IMU M0 (reads calibrated raw & mag data)
    imu_driver_node = Node(
        package='pladypos',
        executable='driver_imu.py',
        name='imu_driver_node',
        namespace=ns,
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port_imu'),
            'baud': LaunchConfiguration('baud')
        }]
    )

    # 5. Madgwick Filter for IMU Fusion (publishes fused orientation on imu/data)
    madgwick_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        namespace=ns,
        output='screen',
        parameters=[{
            'use_mag': True,
            'publish_tf': False,
            'world_frame': 'enu',
            'use_magnetic_field_msg': True,
            'gain': 0.1,
            'zeta': 0.0
        }],
        remappings=[
            ('imu/data_raw', 'imu/data_raw'),
            ('imu/mag', 'imu/mag'),
            ('imu/data', 'imu/data')
        ]
    )

    # Start Madgwick filter once the IMU driver process starts
    filter_start_event = RegisterEventHandler(
        OnProcessStart(
            target_action=imu_driver_node,
            on_start=[madgwick_filter_node]
        )
    )

    return LaunchDescription([
        baud_arg,
        port_imu_arg,
        port_arduino_arg,
        use_joy_arg,
        serial_bridge_node,
        teleop_mixer_node,
        joy_node,
        imu_driver_node,
        filter_start_event
    ])