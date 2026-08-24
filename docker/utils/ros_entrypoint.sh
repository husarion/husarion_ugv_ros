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

# Container PID 1 gets a signal only when it has installed a handler for it -
# the kernel drops everything else, default dispositions included. ros2 launch
# has one for SIGINT, which Python installs itself, but none for SIGTERM: that
# callback is registered with launch's own async signal manager, which never
# reaches signal.signal. So as PID 1 the launch ignored every stop request and
# died on the SIGKILL that ends the systemd stop timeout, with no e-stop, no
# CANopen deconfig and no rmw teardown - which is also how nodes leave their
# /dev/shm segments behind. Supervise the launch instead of exec'ing it and
# turn a SIGTERM into the SIGINT it does handle. An interactive run still
# execs - there the terminal delivers ctrl-c, and a supervised child would
# lose the foreground.
if [ "$$" -ne 1 ] || [ -t 0 ]; then
    exec "$@"
fi

# Job control has to be on or the forwarded SIGINT is discarded as well: bash
# sets SIGINT to SIG_IGN in an asynchronous child, and Python keeps an
# inherited SIG_IGN instead of installing the handler all of this relies on.
set -m
"$@" &
child=$!

trap 'kill -INT "${child}" 2>/dev/null || true' INT TERM

# A trap makes wait return early, so wait until the child is really gone and
# then exit with its status.
while :; do
    if wait "${child}"; then status=0; else status=$?; fi
    kill -0 "${child}" 2>/dev/null || break
done

exit "${status}"
