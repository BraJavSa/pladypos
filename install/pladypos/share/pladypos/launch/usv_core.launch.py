import os
import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def find_serial_ports():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    by_id_path = '/dev/serial/by-id'
    if os.path.exists(by_id_path):
        for f in os.listdir(by_id_path):
            ports.append(os.path.join(by_id_path, f))
    return sorted(list(set(ports)))

def generate_launch_description():
    detected_ports = find_serial_ports()
    if detected_ports:
        selected_port = detected_ports[0]
    else:
        selected_port = '/dev/ttyACM1'
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate for the IMU serial communication'
    )

    imu_driver_node = Node(
        package='pladypos',
        executable='imu_driver.py',
        name='imu_driver_node',
        output='screen',
        parameters=[{
            'port': selected_port,
            'baud': LaunchConfiguration('baud'),
            'use_ned': True
        }]
    )

    complementary_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node',
        output='screen',
        remappings=[
            ('imu/data_raw', 'imu/data_raw'),
            ('imu/mag', 'imu/mag')
        ]
    )

    return LaunchDescription([
        baud_arg,
        imu_driver_node,
        complementary_filter_node
    ])
