#!/bin/bash

# Copyright 2026 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Tests docker/utils/update_config_directory.sh against a fake install prefix in a temporary
# directory. Needs only bash and coreutils, no container, and works as root or not: chown is
# replaced by a stub that logs its arguments. Run from anywhere:
#   bash docker/test/test_update_config_directory.sh

# The checks are strings that check() evals, so they are single-quoted and use variables that look
# unused.
# shellcheck disable=SC2016,SC2034

set -u

script=$(cd "$(dirname "$0")/../utils" && pwd)/update_config_directory.sh
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

failed=0
check() {
  if eval "$2"; then
    echo "ok   $1"
  else
    echo "FAIL $1"
    failed=1
  fi
}

prefix=$work/prefix
config=$work/config
chown_log=$work/chown.log
export HUSARION_UGV_TEST_CONFIG_DIR=$config
export HUSARION_UGV_TEST_ROBOT_CONFIG_ENV=$work/robot_config.env
export AMENT_PREFIX_PATH=$prefix
mkdir -p "$work/bin"
printf '#!/bin/sh\necho "$@" >> "%s"\n' "$chown_log" > "$work/bin/chown"
chmod +x "$work/bin/chown"
export PATH=$work/bin:$PATH

update() { bash "$script" "$@"; }

image_file() {
  mkdir -p "$(dirname "$prefix/share/$1")"
  echo image > "$prefix/share/$1"
}

reset() {
  rm -rf "$prefix" "$config"
  local pkg
  for pkg in husarion_ugv_teleop husarion_ugv_controller husarion_ugv_lights \
    husarion_ugv_localization husarion_ugv_manager husarion_ugv_description; do
    mkdir -p "$prefix/share/ament_index/resource_index/packages"
    : > "$prefix/share/ament_index/resource_index/packages/$pkg"
  done
  image_file husarion_ugv_teleop/config/joy2twist_panther.yaml
  image_file husarion_ugv_teleop/config/joy2twist_lynx.yaml
  image_file husarion_ugv_controller/config/WH01_controller.yaml
  image_file husarion_ugv_controller/config/sub/nested.yaml
  image_file husarion_ugv_lights/config/user_animations.yaml
  image_file husarion_ugv_lights/config/panther_animations.yaml
  image_file husarion_ugv_lights/config/lynx_animations.yaml
  image_file husarion_ugv_localization/config/enu_localization.yaml
  image_file husarion_ugv_localization/config/nmea_navsat.yaml
  image_file husarion_ugv_manager/behavior_trees/lights.xml
  image_file husarion_ugv_manager/behavior_trees/LightsBT.btproj
  image_file husarion_ugv_manager/config/shutdown_hosts.yaml
  image_file husarion_ugv_description/config/components.yaml
  echo "ROBOT_MODEL_NAME=${1:-panther}" > "$HUSARION_UGV_TEST_ROBOT_CONFIG_ENV"
  : > "$chown_log"
}

files() { (cd "$config" && find . -path ./common -prune -o -type f -print | sort); }
snapshot() { (cd "$config" && find . -printf '%p %s %T@\n' | sort); }

# An empty config directory (apart from common/) gets the same files as a full refresh.
reset
mkdir -p "$config/common"
echo user > "$config/common/.env"
update --missing-only > /dev/null
missing_files=$(files)
check "empty: model's joy2twist copied" '[ -f "$config/husarion_ugv_teleop/config/joy2twist_panther.yaml" ]'
check "empty: other model's joy2twist not copied" '[ ! -e "$config/husarion_ugv_teleop/config/joy2twist_lynx.yaml" ]'
check "empty: nested file copied" '[ -f "$config/husarion_ugv_controller/config/sub/nested.yaml" ]'
check "empty: nmea_navsat.yaml not created" '[ ! -e "$config/husarion_ugv_localization/config/nmea_navsat.yaml" ]'
check "empty: common not re-owned" '! grep -q common "$chown_log"'
check "empty: config directory re-owned" 'grep -q "^1000:1001 $config " "$chown_log"'
check "empty: common left alone" '[ "$(cat "$config/common/.env")" = user ]'
reset
mkdir -p "$config/common"
echo user > "$config/common/.env"
update > /dev/null
check "empty: same files as a full refresh" '[ "$missing_files" = "$(files)" ]'

# A second run writes nothing, prints nothing and re-owns nothing.
: > "$chown_log"
before=$(snapshot)
sleep 0.01
out=$(update --missing-only 2>&1)
check "second run: no output" '[ -z "$out" ]'
check "second run: no chown" '[ ! -s "$chown_log" ]'
check "second run: tree unchanged" '[ "$before" = "$(snapshot)" ]'

# Edits survive, a user nmea_navsat.yaml survives, and a file new in the image is added.
echo edited > "$config/husarion_ugv_controller/config/WH01_controller.yaml"
echo user > "$config/husarion_ugv_localization/config/nmea_navsat.yaml"
mkdir -p "$config/other_tool"
image_file husarion_ugv_controller/config/new_file.yaml
: > "$chown_log"
out=$(update --missing-only)
check "new file: copied" '[ -f "$config/husarion_ugv_controller/config/new_file.yaml" ]'
check "new file: logged by name" \
  '[ "$out" = "Copied missing $config/husarion_ugv_controller/config/new_file.yaml from the image defaults" ]'
check "new file: edit kept" '[ "$(cat "$config/husarion_ugv_controller/config/WH01_controller.yaml")" = edited ]'
check "new file: user nmea_navsat.yaml kept" '[ "$(cat "$config/husarion_ugv_localization/config/nmea_navsat.yaml")" = user ]'
check "new file: only it and the config directory re-owned" \
  '[ "$(cat "$chown_log")" = "1000:1001 $config $config/husarion_ugv_controller/config/new_file.yaml" ]'

# A dangling symlink counts as an existing file.
rm "$config/husarion_ugv_manager/config/shutdown_hosts.yaml"
ln -s /nonexistent "$config/husarion_ugv_manager/config/shutdown_hosts.yaml"
update --missing-only > /dev/null
check "dangling symlink: kept" '[ -L "$config/husarion_ugv_manager/config/shutdown_hosts.yaml" ]'

# A deleted file comes back.
rm "$config/husarion_ugv_lights/config/user_animations.yaml"
update --missing-only > /dev/null
check "deleted file: restored" '[ -f "$config/husarion_ugv_lights/config/user_animations.yaml" ]'

# A model change adds the other model's files.
echo ROBOT_MODEL_NAME=lynx > "$HUSARION_UGV_TEST_ROBOT_CONFIG_ENV"
update --missing-only > /dev/null
check "model change: joy2twist added" '[ -f "$config/husarion_ugv_teleop/config/joy2twist_lynx.yaml" ]'
check "model change: animations added" '[ -f "$config/husarion_ugv_lights/config/lynx_animations.yaml" ]'
check "model change: edit kept" '[ "$(cat "$config/husarion_ugv_controller/config/WH01_controller.yaml")" = edited ]'

# A missing directory is created and re-owned along with its files.
rm -rf "$config/husarion_ugv_controller/config/sub"
: > "$chown_log"
update --missing-only > /dev/null
check "missing directory: recreated" '[ -f "$config/husarion_ugv_controller/config/sub/nested.yaml" ]'
check "missing directory: directory and file re-owned" \
  'grep -q "$config/husarion_ugv_controller/config/sub $config/husarion_ugv_controller/config/sub/nested.yaml" "$chown_log"'

# A full refresh overwrites, removes nmea_navsat.yaml and leaves other directories alone.
: > "$chown_log"
update > /dev/null 2>&1
check "full: edit overwritten" '[ "$(cat "$config/husarion_ugv_controller/config/WH01_controller.yaml")" = image ]'
check "full: nmea_navsat.yaml removed" '[ ! -e "$config/husarion_ugv_localization/config/nmea_navsat.yaml" ]'
check "full: other directory not re-owned" '! grep -q other_tool "$chown_log"'
check "full: config directory re-owned without recursion" 'grep -qx "1000:1001 $config" "$chown_log"'
check "full: each package directory re-owned once" \
  '[ "$(grep -o husarion_ugv_manager "$chown_log" | wc -l)" -eq 1 ]'

# Arguments other than --missing-only are refused.
update --missing_only 2> /dev/null
rc=$?
check "typo refused with 2" '[ $rc -eq 2 ]'
update --missing-only extra 2> /dev/null
rc=$?
check "second argument refused with 2" '[ $rc -eq 2 ]'

# A package directory that cannot be created (a file is in the way, which also stops root) does not
# stop the copy of the other packages.
reset
mkdir -p "$config"
echo user > "$config/husarion_ugv_controller"
out=$(update --missing-only 2>&1)
rc=$?
check "blocked directory: exit 0" '[ $rc -eq 0 ]'
check "blocked directory: warning" 'echo "$out" | grep -q "Could not create"'
check "blocked directory: file in the way kept" '[ "$(cat "$config/husarion_ugv_controller")" = user ]'
check "blocked directory: other packages copied" '[ -f "$config/husarion_ugv_teleop/config/joy2twist_panther.yaml" ]'

# A write that fails part way (a full disk) leaves no file behind, so the next run copies it.
reset
mkdir -p "$config"
cat > "$work/bin/cp" << 'STUB'
#!/bin/bash
case "$3" in
  */.components.yaml.update_config)
    echo partial > "$3"
    exit 1
    ;;
esac
exec /bin/cp "$@"
STUB
chmod +x "$work/bin/cp"
components=$config/husarion_ugv_description/config/components.yaml
out=$(update --missing-only 2>&1)
rc=$?
rm "$work/bin/cp"
check "failed write: exit 0" '[ $rc -eq 0 ]'
check "failed write: warning" 'echo "$out" | grep -q "Could not copy .*components.yaml"'
check "failed write: no partial file" '[ ! -e "$components" ]'
check "failed write: no temporary file" '[ -z "$(find "$config" -name "*.update_config")" ]'
check "failed write: other files copied" '[ -f "$config/husarion_ugv_teleop/config/joy2twist_panther.yaml" ]'
: > "$chown_log"
update --missing-only > /dev/null
check "failed write: next run copies it" '[ "$(cat "$components")" = image ]'
check "failed write: next run re-owns it" 'grep -q "$components" "$chown_log"'

# A source file missing from the image is named.
reset
out=$(echo ROBOT_MODEL_NAME=unknown > "$HUSARION_UGV_TEST_ROBOT_CONFIG_ENV" && update --missing-only 2>&1)
check "missing source: named" 'echo "$out" | grep -q "Not in the image: .*joy2twist_unknown.yaml"'

# A package missing from the ament index is named.
reset
rm "$prefix/share/ament_index/resource_index/packages/husarion_ugv_description"
out=$(update --missing-only 2>&1)
check "not found: package named" 'echo "$out" | grep -q "Package not found: husarion_ugv_description"'

exit $failed
