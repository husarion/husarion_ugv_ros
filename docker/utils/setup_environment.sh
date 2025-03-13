#!/bin/bash
set -e

# source robot environment
source /run/husarion/robot_config.env

# check if /config directory is empty except it may contain common directory inside
if [ ! "$(ls -A /config | grep -v common)" ]; then
    echo "Config directory is empty, copying files."
    update_config_directory
fi
