from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ekf_config = os.path.join(
        get_package_share_directory('pladypos'),
        'config', 'ekf.yaml'
    )

    # 1. Detección de AprilTag → publica /usv5/odom (pose raw + covarianza) + TF
    apriltag_node = Node(
        package='pladypos',
        executable='apriltag_pose.py',
        name='apriltag_pose',
        output='screen',
    )

    # 2. EKF: fusiona /usv5/odom (tag) + /imu/data (opcional)
    #    → publica /odometry/filtered + TF odom→usv5 suavizado
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_usv5',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/usv5/odom_filtered'),
        ]
    )

    return LaunchDescription([
        apriltag_node,
        ekf_node,
    ])
