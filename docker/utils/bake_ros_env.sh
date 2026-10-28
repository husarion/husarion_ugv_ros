#!/bin/bash
# Image build step: record what sourcing the ROS setup files adds to the image
# environment and write it as a bash function the entrypoint calls instead.
# Sourcing runs several Python helpers (the ament and colcon setup utils and
# both argcomplete hooks) and costs seconds on a Raspberry Pi at every
# container start, for a result that only depends on the install.
#
# Only install paths are baked. The function applies them only while every
# variable it would set still holds its build-time value, and returns 1
# otherwise so the entrypoint sources the setup files as before. A runtime
# PYTHONPATH or ROS_DISTRO therefore keeps exactly today's behaviour, and the
# RMW, domain, discovery and logging settings are never written at all.
set -euo pipefail

out=$1
distro=${ROS_DISTRO:?}

# Install-path variables the setup files compute. Anything else showing up in
# the diff fails the build, so a ROS update cannot slip a runtime setting in.
static_vars=(
    AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PATH PYTHONPATH
    LD_LIBRARY_PATH PKG_CONFIG_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
    GAZEBO_MODEL_PATH GAZEBO_RESOURCE_PATH GZ_CONFIG_PATH GZ_SIM_RESOURCE_PATH
    IGN_GAZEBO_MODEL_PATH IGN_GAZEBO_RESOURCE_PATH
)
# ros_environment sets this only when it is unset or empty (set-if-unset in
# its dsv), so a value given at container start always wins.
default_vars=(ROS_AUTOMATIC_DISCOVERY_RANGE)
# Read by the setup files without being set by them.
input_vars=(
    AMENT_TRACE_SETUP_FILES AMENT_PYTHON_EXECUTABLE AMENT_SHELL
    AMENT_CURRENT_PREFIX COLCON_TRACE COLCON_PYTHON_EXECUTABLE
    COLCON_CURRENT_PREFIX
)

source_ros() {
    source "/opt/ros/${distro}/setup.bash"
    if [ -f /ros2_ws/install/setup.bash ]; then
        source /ros2_ws/install/setup.bash
    fi
}

dump_env() {
    env -0 | sort -z | tr '\0' '\n' | grep -vE '^(_|SHLVL|OLDPWD)='
}

unset_mark=$'\x01unset'
declare -A before after
while IFS= read -r -d '' kv; do before[${kv%%=*}]=${kv#*=}; done < <(env -0)
after_env=$(set +u +o pipefail; source_ros >/dev/null; env -0 | base64 -w0)
while IFS= read -r -d '' kv; do after[${kv%%=*}]=${kv#*=}; done < <(echo "$after_env" | base64 -d)

is_in() {
    local x=$1
    shift
    local v
    for v in "$@"; do [ "$v" = "$x" ] && return 0; done
    return 1
}

shell_vars=(_ SHLVL OLDPWD PWD)
changed=()
for name in "${!after[@]}"; do
    is_in "$name" "${shell_vars[@]}" && continue
    [ "${before[$name]-$unset_mark}" = "${after[$name]}" ] && continue
    is_in "$name" "${static_vars[@]}" "${default_vars[@]}" || {
        echo "bake_ros_env: sourcing ROS sets ${name}, which is not an install path" >&2
        exit 1
    }
    changed+=("$name")
done
for name in "${!before[@]}"; do
    is_in "$name" "${shell_vars[@]}" && continue
    [ -n "${after[$name]+x}" ] || {
        echo "bake_ros_env: sourcing ROS unsets ${name}" >&2
        exit 1
    }
done
mapfile -t changed < <(printf '%s\n' "${changed[@]}" | sort)

{
    echo "# Generated at image build by bake_ros_env.sh - do not edit."
    echo "ros_env_baked() {"
    for name in ROS_DISTRO "${input_vars[@]}"; do
        printf '    [ "${%s-%q}" = %q ] || return 1\n' \
            "$name" "$unset_mark" "${before[$name]-$unset_mark}"
    done
    for name in "${changed[@]}"; do
        is_in "$name" "${default_vars[@]}" && continue
        printf '    [ "${%s-%q}" = %q ] || return 1\n' \
            "$name" "$unset_mark" "${before[$name]-$unset_mark}"
    done
    for name in "${changed[@]}"; do
        if is_in "$name" "${default_vars[@]}"; then
            printf '    [ -n "${%s-}" ] || export %s=%q\n' "$name" "$name" "${after[$name]}"
        else
            printf '    export %s=%q\n' "$name" "${after[$name]}"
        fi
    done
    echo "}"
} > "$out"

# The baked environment has to be the one sourcing produces, with and without
# a discovery range given at start, or the image is not built.
for range in unset '' LOCALHOST; do
    sourced=$(
        set +u +o pipefail
        [ "$range" = unset ] || export ROS_AUTOMATIC_DISCOVERY_RANGE=$range
        source_ros >/dev/null
        dump_env
    )
    baked=$(
        [ "$range" = unset ] || export ROS_AUTOMATIC_DISCOVERY_RANGE=$range
        source "$out"
        ros_env_baked
        dump_env
    )
    if [ "$sourced" != "$baked" ]; then
        echo "bake_ros_env: baked environment differs from the sourced one" >&2
        diff <(echo "$sourced") <(echo "$baked") >&2 || true
        exit 1
    fi
done
