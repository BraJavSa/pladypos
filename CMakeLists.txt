cmake_minimum_required(VERSION 3.8)
project(pladypos)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/python_nodes/imu_driver.py
  src/python_nodes/serial_bridge.py
  src/python_nodes/teleop_mixer.py
  src/python_nodes/motor_mapper.py
  src/python_nodes/camera_driver.py
  src/python_nodes/kinect_to_v4l2.py
  src/python_nodes/depth_to_mono8.py
  src/python_nodes/pose_imu_odometry.py
  src/python_nodes/apriltag_pose.py
  src/python_nodes/record_orientation.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
