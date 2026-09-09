import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

# Puerto fijo de la IMU (no se expone como argumento de launch)
IMU_PORT = '/dev/ttyACM0'
IMU_BAUD = 115200


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

    imu_driver_node = Node(
        package='pladypos',
        executable='driver_imu.py',
        name='imu_driver_node',
        namespace=ns,
        output='screen',
        parameters=[{
            'port': IMU_PORT,
            'baud': IMU_BAUD
        }]
    )

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

    filter_start_event = RegisterEventHandler(
        OnProcessStart(
            target_action=imu_driver_node,
            on_start=[madgwick_filter_node]
        )
    )

    return LaunchDescription([
        imu_driver_node,
        filter_start_event
    ])