#!/bin/bash
set -e

# source robot environment
source /run/husarion/robot_config.env

# Copy the driver files that are missing from /config: the first run, a robot model change, or a
# file that a driver update added. An existing file is never overwritten, and a start with nothing
# missing writes nothing. Skipped when the command is the full refresh itself. Sourced by the
# entrypoint, so $1 is the container command.
if [ "${1-}" != update_config_directory ]; then
    update_config_directory --missing-only || echo "Copying or re-owning the missing driver config files failed, continuing." >&2
fi
