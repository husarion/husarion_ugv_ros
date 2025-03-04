#!/bin/bash
set -e

# method to copy directory/file using package name
copy_package_file() {
  local package_name=$1
  local file_name=$2
  local package_file_path=$(ros2 pkg prefix $package_name)/share/$package_name/$file_name

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

copy_package_file husarion_ugv_controller config
copy_package_file husarion_ugv_lights config/user_animations.yaml
copy_package_file husarion_ugv_localization config
rm /config/husarion_ugv_localization/config/nmea_navsat.yaml || true
copy_package_file husarion_ugv_manager behavior_trees/lights.xml
copy_package_file husarion_ugv_manager behavior_trees/LightsBT.btproj
copy_package_file husarion_ugv_manager config/shutdown_hosts.yaml
copy_package_file husarion_ugv_description config/components.yaml

# Change ownership of the copied files to host user
chown -R 1000:1001 /config
