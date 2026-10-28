#!/bin/bash
set -e

# Same lookup as `ros2 pkg prefix`: the first prefix in AMENT_PREFIX_PATH whose
# ament index lists the package. Each ros2 CLI call is a Python cold start of
# about a second on the robot, and this runs ten of them on a first boot.
package_prefix() {
  local prefix
  local IFS=:
  for prefix in $AMENT_PREFIX_PATH; do
    if [ -n "$prefix" ] && [ -f "$prefix/share/ament_index/resource_index/packages/$1" ]; then
      echo "$prefix"
      return 0
    fi
  done
  echo "Package not found" >&2
  return 1
}

# method to copy directory/file using package name
copy_package_file() {
  local package_name=$1
  local file_name=$2
  local package_file_path=$(package_prefix $package_name)/share/$package_name/$file_name

  if [ ! -d "/config/$package_name" ]; then
    mkdir -p /config/$package_name
  fi

  # create subdirectories for files with subdirectories
  if [[ $file_name == */* ]]; then
    local subdirectory=$(dirname $file_name)
    if [ ! -d "/config/$package_name/$subdirectory" ]; then
      mkdir -p /config/$package_name/$subdirectory
    fi
  fi

  if [ -d "$package_file_path" ]; then
    cp -rL $package_file_path /config/$package_name || true
  else
    cp -L $package_file_path /config/$package_name/$file_name || true
  fi
}

# source robot environment
source /run/husarion/robot_config.env

copy_package_file husarion_ugv_teleop config/joy2twist_${ROBOT_MODEL_NAME}.yaml
copy_package_file husarion_ugv_controller config
copy_package_file husarion_ugv_lights config/user_animations.yaml
copy_package_file husarion_ugv_lights config/${ROBOT_MODEL_NAME}_animations.yaml
copy_package_file husarion_ugv_localization config
rm /config/husarion_ugv_localization/config/nmea_navsat.yaml || true
copy_package_file husarion_ugv_manager behavior_trees/lights.xml
copy_package_file husarion_ugv_manager behavior_trees/LightsBT.btproj
copy_package_file husarion_ugv_manager config/shutdown_hosts.yaml
copy_package_file husarion_ugv_description config/components.yaml

# Change ownership of the copied files to host user
chown -R 1000:1001 /config
