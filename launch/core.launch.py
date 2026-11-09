import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Ultra-lightweight onboard launch file for the USV NUC (pladypos-usv5).
    Runs ONLY hardware I/O:
      1. serial_bridge.py: receives control actions (/usv5/pwm_out) from the server
         or controller and drives the Arduino Micro thrusters.
      2. driver_imu.py + imu_filter_madgwick_node: publishes /usv5/imu/data to the server.
    Locks DDS to LOCALHOST so all cross-machine communication flows strictly over Zenoh TCP.
    """
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

    # 1. Serial Bridge to Arduino Micro (receives /usv5/pwm_out control actions and sends PWM to thrusters)
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

    # 2. IMU Driver for SparkFun 9DoF Razor IMU M0 (reads calibrated raw & mag data)
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

    # 3. Madgwick Filter for IMU Orientation (publishes /usv5/imu/data at 100 Hz)
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
            'zeta': 0.0,
            'orientation_stddev': 0.05
        }],
        remappings=[
            ('imu/data_raw', 'imu/data_raw'),
            ('imu/mag', 'imu/mag'),
            ('imu/data', 'imu/data')
        ]
    )

    filter_start_event = RegisterEventHandler(
        OnProcessStart(
            target_action=imu_driver_node,
            on_start=[madgwick_filter_node]
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        baud_arg,
        port_imu_arg,
        port_arduino_arg,
        serial_bridge_node,
        imu_driver_node,
        filter_start_event,
    ])
