#!/usr/bin/env bash
# Host-side teleop for the simulated Panther.
# Requires: ros-jazzy-ros-base, ros-jazzy-rmw-cyclonedds-cpp,
#           ros-jazzy-teleop-twist-keyboard.
set -e

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

NAMESPACE=${ROBOT_NAMESPACE:-panther}

ros2 service call "/${NAMESPACE}/hardware/e_stop_reset" std_srvs/srv/Trigger >/dev/null

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r __ns:="/${NAMESPACE}" \
  -r cmd_vel:="/${NAMESPACE}/cmd_vel" \
  -p stamped:=true \
  -p frame_id:=base_link
