#!/bin/bash
# The ros-core base entrypoint only sources /opt/ros - source the built
# workspace and the Husarion robot environment too.
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# export the robot env to the launched process (populates /config on first run)
set -a
source /usr/local/sbin/setup_environment
set +a

exec "$@"
