from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    """
    Server launch file on rawfie (inside ros2_streaming_container):
      - Isolates server DDS to ROS_DOMAIN_ID=10 so camera_0354 and DDS discovery NEVER
        collide with USV (ROS_DOMAIN_ID=5) on the 10.0.10.0/16 subnet.
      1. Strictly ONLY camera .204 (camera_0354)
      2. web_video_server (port 8082) + stream_guard.py (port 8083)
      3. visual_tag_estimator.py -> /usv5/visual_odom + TF camera -> usv5_visual
      4. static_transform_publisher -> TF camera -> odom
      5. direct_usv_fusion.py -> fuses /usv5/visual_odom + /usv5/imu/data (or camera-only fallback
         when IMU is absent) -> /usv5/odometry/filtered + TF odom -> usv5_odom
      6. teleop_mixer.py -> maps /usv5/joy (from laptop) -> /usv5/pwm_out (to USV NUC)
      7. pool_boundary_publisher.py -> publishes pool walls & 70% Figure-8 on /pool_boundary
      8. zenoh-bridge-ros2dds router on tcp/0.0.0.0:7447 (-d 10 --no-multicast-scouting)
    """
    active_cam_id = '0354'
    calib = '/config/calibration_536890354.yaml'

    nodes = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', '10'),
    ]

    # 1. Strictly Camera .204 (0354) ONLY
    driver_node = Node(
        package='camera_aravis2',
        executable='camera_driver_gv',
        name='camera_0354',
        namespace='camera_0354',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'guid': '10.250.253.204',
            'frame_id': '0354',
            'camera_info_urls': [f'file://{calib}'],
            'TransportLayerControl': {
                'GevSCPSPacketSize': 1440
            },
            'ImageFormatControl': {
                'PixelFormat': 'BayerGB8',
                'Width': 2048,
                'Height': 2048
            },
            'AcquisitionControl': {
                'AcquisitionMode': 'Continuous'
            }
        }]
    )
    nodes.append(driver_node)

    # 2. Internal web_video_server (port 8082) + Stream Guard Proxy (port 8083)
    web_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': 8082}]
    )
    nodes.append(web_server_node)

    stream_guard = ExecuteProcess(
        cmd=['python3', '/config/stream_guard.py'],
        output='screen',
        emulate_tty=True
    )
    nodes.append(stream_guard)

    # 3. AprilTag Visual Estimator on rawfie (reads localhost:8083 -> /usv5/visual_odom + TF camera -> usv5_visual)
    visual_tag_estimator = ExecuteProcess(
        cmd=[
            'python3', '/config/visual_tag_estimator.py',
            '--ros-args',
            '-p', f'stream_url:=http://127.0.0.1:8083/stream?topic=/camera_{active_cam_id}/camera_{active_cam_id}/image_raw',
            '-p', f'calib_url:=http://127.0.0.1:8083/calibration?camera={active_cam_id}',
            '-p', 'tag_id:=285',
            '-p', 'tag_size:=0.25',
            '-p', 'odom_topic:=/usv5/visual_odom',
            '-p', 'tag_detected_topic:=/usv5/tag_detected',
            '-p', 'publish_tf:=false',
            '-p', 'camera_frame:=camera',
            '-p', 'child_frame:=usv5_visual'
        ],
        output='screen',
        emulate_tty=True
    )
    nodes.append(visual_tag_estimator)

    # 4. Static TF: camera -> odom (water_depth = 5.1 m, Rx(pi))
    static_tf_camera_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camera_to_odom',
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
            '--child-frame-id', 'odom'
        ]
    )
    nodes.append(static_tf_camera_to_odom)

    # 5. Direct USV Fusion on rawfie (Camera+IMU fusion with automatic Camera-Only fallback)
    direct_usv_fusion = ExecuteProcess(
        cmd=[
            'python3', '/config/direct_usv_fusion.py',
            '--ros-args',
            '-p', 'visual_odom_topic:=/usv5/visual_odom',
            '-p', 'imu_topic:=/usv5/imu/data',
            '-p', 'odom_topic:=/usv5/odometry/filtered',
            '-p', 'frame_id:=odom',
            '-p', 'child_frame_id:=usv5_odom',
            '-p', 'water_depth:=5.1',
            '-p', 'alpha_pos:=0.45',
            '-p', 'yaw_corr_rate:=0.08',
            '-p', 'imu_timeout_sec:=0.5'
        ],
        output='screen',
        emulate_tty=True
    )
    nodes.append(direct_usv_fusion)

    # 6. Teleoperation Mixer on rawfie (maps /usv5/joy from laptop -> /usv5/pwm_out to USV NUC)
    teleop_mixer = ExecuteProcess(
        cmd=[
            'python3', '/config/teleop_mixer.py',
            '--ros-args',
            '-r', '__ns:=/usv5'
        ],
        output='screen',
        emulate_tty=True
    )
    nodes.append(teleop_mixer)

    # 7. Pool Boundary & Figure-8 Reference Marker Publisher on rawfie
    pool_boundary = ExecuteProcess(
        cmd=[
            'python3', '/config/pool_boundary_publisher.py',
            '--ros-args',
            '-p', 'frame_id:=odom',
            '-p', 'marker_topic:=/pool_boundary'
        ],
        output='screen',
        emulate_tty=True
    )
    nodes.append(pool_boundary)

    # 8. Central Zenoh Router on rawfie (v1.10.2, ROS_DOMAIN_ID=10, --no-multicast-scouting)
    zenoh_router = ExecuteProcess(
        cmd=[
            '/config/zenoh-bridge-ros2dds',
            '-d', '10',
            '--no-multicast-scouting',
            '-c', '/config/zenoh_config.json5',
            '-l', 'tcp/0.0.0.0:7447'
        ],
        output='screen',
        emulate_tty=True
    )
    nodes.append(zenoh_router)

    return LaunchDescription(nodes)
