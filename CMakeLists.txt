cmake_minimum_required(VERSION 3.8)
project(pladypos)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/python_nodes/driver_imu.py
  src/python_nodes/serial_bridge.py
  src/python_nodes/teleop_mixer.py
  src/python_nodes/visual_tag_estimator.py
  src/python_nodes/visual_imu_fusion.py
  src/python_nodes/direct_usv_fusion.py
  src/python_nodes/pool_boundary_publisher.py
  src/python_nodes/inverse_dynamics_controller.py
  src/python_nodes/mpc_controller.py
  src/python_nodes/gcs_error_control_plotter.py
  scripts/start_zenoh_gcs.sh
  scripts/start_zenoh_usv.sh
  scripts/sync_boat_time.sh
  bin/zenoh-bridge-ros2dds
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  scripts
  bin
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
