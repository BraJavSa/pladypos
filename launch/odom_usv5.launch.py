from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Detección de AprilTag → publica /usv5/odom a 20Hz + TF (camera -> usv5)
    apriltag_node = Node(
        package='pladypos',
        executable='apriltag_pose.py',
        name='apriltag_pose',
        output='screen',
    )

    # 2. Transformada TF Estática: pool_base_0352 (5.1m arriba de camera, rotado 180° respecto a Y)
    static_tf_pool_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pool_base_0352',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '5.1',
            '--qx', '0.0',
            '--qy', '1.0',
            '--qz', '0.0',
            '--qw', '0.0',
            '--frame-id', 'camera',
            '--child-frame-id', 'pool_base_0352'
        ]
    )

    return LaunchDescription([
        apriltag_node,
        static_tf_pool_base,
    ])
