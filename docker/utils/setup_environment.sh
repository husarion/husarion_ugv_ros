#!/bin/bash
set -e

# source robot environment
source /run/husarion/robot_config.env

# Check for the actual driver files instead of "empty except common" - /config
# can hold directories that are not ours (the OS keeps rmw profiles there), and
# a card moved to the other robot model is missing its joy2twist config.
if [ ! -d /config/husarion_ugv_controller ] || \
   [ ! -f "/config/husarion_ugv_teleop/config/joy2twist_${ROBOT_MODEL_NAME}.yaml" ]; then
    echo "Config directory is missing driver files, copying."
    update_config_directory
fi
