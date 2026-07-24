import os
import glob
import time
import serial
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def find_devices():
    by_id_path = '/dev/serial/by-id'
    by_id_ports = []
    if os.path.exists(by_id_path):
        for f in os.listdir(by_id_path):
            path = os.path.join(by_id_path, f)
            by_id_ports.append(os.path.realpath(path))
            
    candidates = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    candidates = sorted(list(set([os.path.realpath(c) for c in candidates] + by_id_ports)))
    
    imu_port = None
    arduino_port = None
    
    for port in candidates:
        try:
            ser = serial.Serial(port, 115200, timeout=0.15)
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(bytearray([0xFF, 0xFF, 0xF0, 0x00, 0xF0]))
            time.sleep(0.05)
            buf = ser.read(30)
            ser.close()
            
            if b"PLADYPOS_BRIDGE" in buf:
                arduino_port = port
                break
        except Exception:
            continue
            
    for port in candidates:
        if port == arduino_port:
            continue
        try:
            ser = serial.Serial(port, 115200, timeout=0.2)
            time.sleep(0.05)
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            ser.close()
            if ',' in line and len(line.split(',')) >= 10:
                imu_port = port
                break
        except Exception:
            continue
            
    if not imu_port:
        if os.path.exists(by_id_path):
            for f in os.listdir(by_id_path):
                if 'razor' in f.lower() or 'micro' in f.lower():
                    resolved = os.path.realpath(os.path.join(by_id_path, f))
                    if resolved != arduino_port:
                        imu_port = resolved
                        break
                        
    if not arduino_port:
        for c in candidates:
            if c != imu_port:
                arduino_port = c
                break
                
    if not imu_port:
        imu_port = '/dev/ttyACM1'
    if not arduino_port:
        arduino_port = '/dev/ttyACM0'
        
    return imu_port, arduino_port

def generate_launch_description():
    imu_port, arduino_port = find_devices()
    
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

    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate for the IMU and Arduino communication'
    )

    imu_driver_node = Node(
        package='pladypos',
        executable='imu_driver.py',
        name='imu_driver_node',
        namespace=ns,
        output='screen',
        parameters=[{
            'port': imu_port,
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
            'port': arduino_port,
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

    filter_start_event = RegisterEventHandler(
        OnProcessStart(
            target_action=imu_driver_node,
            on_start=[complementary_filter_node]
        )
    )

    return LaunchDescription([
        baud_arg,
        imu_driver_node,
        serial_bridge_node,
        teleop_mixer_node,
        filter_start_event
    ])
