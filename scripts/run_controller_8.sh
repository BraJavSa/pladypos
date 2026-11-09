#!/usr/bin/env bash
export ROS_DOMAIN_ID=10
source /opt/ros/jazzy/setup.bash
exec python3 /config/inverse_dynamics_controller.py --ros-args \
  -p center_x:=-0.5713 \
  -p center_y:=-1.2731 \
  -p scale_factor:=0.70 \
  -p period_s:=55.0 \
  -p control_rate_hz:=30.0 \
  -p odom_topic:=/usv5/odometry/filtered \
  -p visual_odom_topic:=/usv5/visual_odom \
  -p imu_topic:=/usv5/imu/data \
  -p pwm_topic:=/usv5/pwm_out \
  -p marker_topic:=/pool_boundary \
  -p frame_id:=odom "$@"
