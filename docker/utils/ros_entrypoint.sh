#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
test -f "/ros2_ws/install/setup.bash" && source "/ros2_ws/install/setup.bash"

# Exports the robot environment to the command. setup_environment reads the command as $1.
set -e -a && source setup_environment
# <additional-user-commands>

# Forwards a SIGTERM stop to the command as SIGINT when there is no terminal (see supervise_command).
source supervise_command "$@"
exec "$@"
