from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    visual_tag_node = Node(
        package='pladypos',
        executable='visual_tag_estimator.py',
        name='visual_tag_estimator',
        output='screen',
    )

    visual_imu_node = Node(
        package='pladypos',
        executable='visual_imu_fusion.py',
        name='visual_imu_fusion',
        output='screen',
    )

    static_tf_pool_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pool_base_0352',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '5.1',
            '--qx', '1.0',
            '--qy', '0.0',
            '--qz', '0.0',
            '--qw', '0.0',
            '--frame-id', 'camera',
            '--child-frame-id', 'pool_base_0352'
        ]
    )

    return LaunchDescription([
        visual_tag_node,
        visual_imu_node,
        static_tf_pool_base,
    ])
