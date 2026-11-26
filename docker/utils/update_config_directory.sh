#!/bin/bash
set -e

# Test-only overrides for docker/test/test_update_config_directory.sh.
config_dir=${HUSARION_UGV_TEST_CONFIG_DIR:-/config}
robot_config_env=${HUSARION_UGV_TEST_ROBOT_CONFIG_ENV:-/run/husarion/robot_config.env}

usage() {
  echo "Usage: update_config_directory [--missing-only]" >&2
  echo "  Without an option every driver file in $config_dir is refreshed and edits are overwritten." >&2
  echo "  --missing-only  copy only the files that do not exist in $config_dir, never overwrite one." >&2
}

missing_only=false
case "$#:${1-}" in
  0:) ;;
  1:--missing-only) missing_only=true ;;
  *)
    usage
    exit 2
    ;;
esac

# The only paths re-owned: the package directories of a full refresh, what --missing-only created.
declare -A package_dirs=()
created=()

# Same lookup as `ros2 pkg prefix`: the first prefix in AMENT_PREFIX_PATH whose
# ament index lists the package. Avoids a ros2 CLI (Python) start per file.
package_prefix() {
  local prefix
  local prefixes
  IFS=: read -ra prefixes <<< "${AMENT_PREFIX_PATH-}"
  for prefix in "${prefixes[@]}"; do
    if [ -n "$prefix" ] && [ -f "$prefix/share/ament_index/resource_index/packages/$1" ]; then
      echo "$prefix"
      return 0
    fi
  done
  echo "Package not found: $1" >&2
  return 1
}

# Create a directory and its missing parents, and remember each one created.
make_dir() {
  [ -d "$1" ] && return 0
  make_dir "$(dirname "$1")" || return 1
  if mkdir "$1" 2>/dev/null; then
    created+=("$1")
  else
    [ -d "$1" ]
  fi
}

# Copy the files of a source file or directory that are missing at the destination.
copy_missing() {
  local source=$1
  local destination=$2
  local file
  local target
  local temporary
  if [ ! -e "$source" ]; then
    echo "Not in the image: $source" >&2
    return 0
  fi
  while IFS= read -r -d '' file; do
    target=$destination${file#"$source"}
    if [ -e "$target" ] || [ -L "$target" ]; then
      continue
    fi
    # Read by the launch from the package share, never from the config directory (see
    # nmea_navsat.launch.py), so a copy here would be an edited file that nothing reads.
    [ "$target" != "$nmea_config" ] || continue
    make_dir "$(dirname "$target")" || {
      echo "Could not create the directory of $target" >&2
      continue
    }
    # Moved into place without replacing anything, so a failed write leaves no partial file and a
    # file written since the check is kept. Older mv -n returns 0 when it skips, so the temporary
    # file still existing is the test.
    temporary="$(dirname "$target")/.$(basename "$target").update_config"
    rm -f "$temporary"
    if cp -L "$file" "$temporary" 2>/dev/null && mv -n "$temporary" "$target" 2>/dev/null &&
      [ ! -e "$temporary" ]; then
      created+=("$target")
      echo "Copied missing $target from the image defaults"
    elif [ ! -e "$target" ]; then
      echo "Could not copy $file to $target" >&2
    fi
    rm -f "$temporary"
  done < <(find -L "$source" -type f -print0)
}

# method to copy directory/file using package name
copy_package_file() {
  local package_name=$1
  local file_name=$2
  local prefix
  prefix=$(package_prefix "$package_name") || return 0
  local package_file_path="$prefix/share/$package_name/$file_name"

  if [ "$missing_only" = true ]; then
    copy_missing "$package_file_path" "$config_dir/$package_name/$file_name"
    return 0
  fi

  package_dirs["$config_dir/$package_name"]=1

  if [ ! -d "$config_dir/$package_name" ]; then
    mkdir -p "$config_dir/$package_name"
  fi

  # create subdirectories for files with subdirectories
  if [[ $file_name == */* ]]; then
    local subdirectory
    subdirectory=$(dirname "$file_name")
    if [ ! -d "$config_dir/$package_name/$subdirectory" ]; then
      mkdir -p "$config_dir/$package_name/$subdirectory"
    fi
  fi

  if [ -d "$package_file_path" ]; then
    cp -rL "$package_file_path" "$config_dir/$package_name" || true
  else
    cp -L "$package_file_path" "$config_dir/$package_name/$file_name" || true
  fi
}

# source robot environment
# shellcheck source=/dev/null
source "$robot_config_env"

nmea_config=$config_dir/husarion_ugv_localization/config/nmea_navsat.yaml

copy_package_file husarion_ugv_teleop "config/joy2twist_${ROBOT_MODEL_NAME}.yaml"
copy_package_file husarion_ugv_controller config
copy_package_file husarion_ugv_lights config/user_animations.yaml
copy_package_file husarion_ugv_lights "config/${ROBOT_MODEL_NAME}_animations.yaml"
copy_package_file husarion_ugv_localization config
if [ "$missing_only" = false ]; then
  rm -f "$nmea_config"
fi
copy_package_file husarion_ugv_manager behavior_trees/lights.xml
copy_package_file husarion_ugv_manager behavior_trees/LightsBT.btproj
copy_package_file husarion_ugv_manager config/shutdown_hosts.yaml
copy_package_file husarion_ugv_description config/components.yaml

# The config directory itself too, without recursion: a bind mount of a missing host directory is root's.
if [ "$missing_only" = true ]; then
  [ ${#created[@]} -eq 0 ] || chown 1000:1001 "$config_dir" "${created[@]}"
else
  chown 1000:1001 "$config_dir"
  [ ${#package_dirs[@]} -eq 0 ] || chown -R 1000:1001 "${!package_dirs[@]}"
fi
