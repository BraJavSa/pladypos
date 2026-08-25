import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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

    imu_driver_node = Node(
        package='pladypos',
        executable='imu_driver.py',
        name='imu_driver_node',
        namespace=ns,
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port_imu'),
            'baud': LaunchConfiguration('baud'),
            # Mapeo de ejes sensor -> robot (FLU), determinado a partir del TF:
            # el IMU esta fisicamente montado con +Y_sensor apuntando a proa,
            # +X_sensor apuntando a babor, y Z_sensor invertido respecto a arriba.
            #   X_robot (adelante)  = +Y_sensor
            #   Y_robot (izquierda) = +X_sensor
            #   Z_robot (arriba)    = -Z_sensor
            # YAML interpreta 'y' y 'n' como booleanos. Usaremos 'axis_x', 'axis_y', 'axis_z'
            'map_x_from': 'axis_y',
            'map_y_from': 'axis_x',
            'map_z_from': 'axis_z',
            'sign_x': 1.0,
            'sign_y': 1.0,
            'sign_z': -1.0,
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
        parameters=[{
            'use_mag': True,
            'publish_tf': False
        }],
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
        port_imu_arg,
        port_arduino_arg,
        imu_driver_node,
        serial_bridge_node,
        teleop_mixer_node,
        filter_start_event
    ])