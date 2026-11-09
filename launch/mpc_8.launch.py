import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('pladypos')
    usv_id = 5
    try:
        config_path = os.path.join(pkg_share, 'config', 'usv_config.yaml')
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config_data = yaml.safe_load(f)
                usv_id = config_data.get('usv_id', 5)
    except Exception:
        pass

    ns = f"usv{usv_id}"
    rviz_config = os.path.join(pkg_share, 'config', 'pool.rviz')

    center_x_arg = DeclareLaunchArgument(
        'center_x', default_value='-0.5713', description='Pool geometric center X [m] in odom'
    )
    center_y_arg = DeclareLaunchArgument(
        'center_y', default_value='-1.2731', description='Pool geometric center Y [m] in odom'
    )
    scale_arg = DeclareLaunchArgument(
        'scale_factor', default_value='0.70', description='Fraction of pool workspace (0.70 = 70%)'
    )
    period_arg = DeclareLaunchArgument(
        'period_s', default_value='85.0', description='Figure-8 period [s] (slower & smoother)'
    )
    horizon_arg = DeclareLaunchArgument(
        'horizon_n', default_value='15', description='MPC prediction horizon steps N (default: 15)'
    )
    dt_mpc_arg = DeclareLaunchArgument(
        'dt_mpc', default_value='0.12', description='MPC prediction step [s] (15 * 0.12 = 1.8 s lookahead)'
    )
    start_closest_arg = DeclareLaunchArgument(
        'start_at_closest', default_value='true', description='Start trajectory at closest point on Figure-8'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false', description='Set true only if launching on GCS laptop with display'
    )

    mpc_node = Node(
        package='pladypos',
        executable='mpc_controller.py',
        name='mpc_controller',
        output='screen',
        parameters=[{
            'center_x': LaunchConfiguration('center_x'),
            'center_y': LaunchConfiguration('center_y'),
            'scale_factor': LaunchConfiguration('scale_factor'),
            'period_s': LaunchConfiguration('period_s'),
            'horizon_n': LaunchConfiguration('horizon_n'),
            'dt_mpc': LaunchConfiguration('dt_mpc'),
            'start_at_closest': LaunchConfiguration('start_at_closest'),
            'control_rate_hz': 30.0,
            'odom_topic': f'/{ns}/odometry/filtered',
            'visual_odom_topic': f'/{ns}/visual_odom',
            'imu_topic': f'/{ns}/imu/data',
            'pwm_topic': f'/{ns}/pwm_out',
            'marker_topic': '/pool_boundary',
            'frame_id': 'odom',
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '5'),
        center_x_arg,
        center_y_arg,
        scale_arg,
        period_arg,
        horizon_arg,
        dt_mpc_arg,
        start_closest_arg,
        use_rviz_arg,
        mpc_node,
        rviz_node,
    ])

