cmake_minimum_required(VERSION 3.8)
project(pladypos)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  src/python_nodes/usv_controller.py
  src/python_nodes/thruster_allocation.py
  src/python_nodes/imu_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
