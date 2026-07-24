import os
import glob
import time
import serial
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def find_imu_port():
    by_id_path = '/dev/serial/by-id'
    by_id_ports = []
    if os.path.exists(by_id_path):
        for f in os.listdir(by_id_path):
            path = os.path.join(by_id_path, f)
            by_id_ports.append(os.path.realpath(path))
            
    candidates = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    candidates = sorted(list(set([os.path.realpath(c) for c in candidates] + by_id_ports)))
    
    for port in candidates:
        try:
            ser = serial.Serial(port, 115200, timeout=0.2)
            time.sleep(0.05)
            for _ in range(3):
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if ',' in line and len(line.split(',')) >= 10:
                    ser.close()
                    return port
            ser.close()
        except Exception:
            continue
            
    if os.path.exists(by_id_path):
        for f in os.listdir(by_id_path):
            if any(keyword in f.lower() for keyword in ['arduino', 'micro', 'razor']):
                return os.path.realpath(os.path.join(by_id_path, f))
                
    if candidates:
        return candidates[0]
        
    return '/dev/ttyACM1'

def generate_launch_description():
    selected_port = find_imu_port()
    
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
