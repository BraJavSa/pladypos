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
        'period_s', default_value='120.0', description='Archimedean Spiral period [s] (2 turns per period)'
    )
    k_pos_arg = DeclareLaunchArgument(
        'k_pos', default_value='0.65', description='Outer-loop longitudinal (surge) position tracking gain'
    )
    k_pos_v_arg = DeclareLaunchArgument(
        'k_pos_v', default_value='0.45', description='Outer-loop lateral (sway) position tracking gain'
    )
    k_psi_arg = DeclareLaunchArgument(
        'k_psi', default_value='0.90', description='Outer-loop yaw heading tracking gain'
    )
    lambda_u_arg = DeclareLaunchArgument(
        'lambda_u', default_value='0.70', description='Surge sliding surface linear slope'
    )
    lambda_v_arg = DeclareLaunchArgument(
        'lambda_v', default_value='0.50', description='Sway sliding surface linear slope'
    )
    lambda_r_arg = DeclareLaunchArgument(
        'lambda_r', default_value='1.20', description='Yaw sliding surface linear slope'
    )
    k_s_u_arg = DeclareLaunchArgument(
        'k_s_u', default_value='0.09', description='Surge SMC switching gain'
    )
    k_s_v_arg = DeclareLaunchArgument(
        'k_s_v', default_value='0.05', description='Sway SMC switching gain'
    )
    k_s_r_arg = DeclareLaunchArgument(
        'k_s_r', default_value='0.15', description='Yaw SMC switching gain'
    )
    max_tau_u_arg = DeclareLaunchArgument(
        'max_tau_u', default_value='14.0', description='Max longitudinal (surge) force [N]'
    )
    max_tau_v_arg = DeclareLaunchArgument(
        'max_tau_v', default_value='8.5', description='Max lateral (sway) force [N]'
    )
    max_tau_r_arg = DeclareLaunchArgument(
        'max_tau_r', default_value='6.0', description='Max yaw torque [N*m]'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false', description='Set true only if launching on GCS laptop with display'
    )

    controller_node = Node(
        package='pladypos',
        executable='smc_controller.py',
        name='smc_controller',
        output='screen',
        parameters=[{
            'trajectory_type': 'spiral',
            'enable_obstacles': False,
            'center_x': LaunchConfiguration('center_x'),
            'center_y': LaunchConfiguration('center_y'),
            'scale_factor': LaunchConfiguration('scale_factor'),
            'period_s': LaunchConfiguration('period_s'),
            'k_pos': LaunchConfiguration('k_pos'),
            'k_pos_v': LaunchConfiguration('k_pos_v'),
            'k_psi': LaunchConfiguration('k_psi'),
            'lambda_u': LaunchConfiguration('lambda_u'),
            'lambda_v': LaunchConfiguration('lambda_v'),
            'lambda_r': LaunchConfiguration('lambda_r'),
            'k_s_u': LaunchConfiguration('k_s_u'),
            'k_s_v': LaunchConfiguration('k_s_v'),
            'k_s_r': LaunchConfiguration('k_s_r'),
            'max_tau_u': LaunchConfiguration('max_tau_u'),
            'max_tau_v': LaunchConfiguration('max_tau_v'),
            'max_tau_r': LaunchConfiguration('max_tau_r'),
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
        k_pos_arg,
        k_pos_v_arg,
        k_psi_arg,
        lambda_u_arg,
        lambda_v_arg,
        lambda_r_arg,
        k_s_u_arg,
        k_s_v_arg,
        k_s_r_arg,
        max_tau_u_arg,
        max_tau_v_arg,
        max_tau_r_arg,
        use_rviz_arg,
        controller_node,
        rviz_node,
    ])
