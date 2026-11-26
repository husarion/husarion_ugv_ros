# shellcheck shell=bash

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

# Sourced by the image entrypoint with the command it is about to exec, as in
# `source supervise_command "$@"` right before `exec "$@"`. Without a terminal on stdin it runs the
# command as a child, turns SIGTERM and SIGINT into SIGINT for it, and exits with its status. With a
# terminal it returns and the entrypoint execs, because an interactive command must stay in the
# terminal's foreground process group. So `docker run -t` and compose `tty: true` keep a plain exec,
# which ignores a SIGTERM stop: use STOPSIGNAL SIGINT (set in the image) with them. Other signals
# (SIGHUP, SIGQUIT, SIGUSR1/2) reach this shell, not the command.

if [ ! -t 0 ]; then
    # The entrypoint sources this under set -a. Its variables must not reach the command's env.
    set +a

    # Set before the child starts. A signal that arrives before its pid is known is kept and
    # forwarded right after. The child does not inherit the trap.
    child=
    pending=
    trap 'if [ -n "${child}" ]; then kill -INT "${child}" 2>/dev/null || true; else pending=1; fi' INT TERM

    # With job control off bash starts the child with SIGINT ignored, and Python keeps that.
    # Off again once it runs, or bash prints a job status line when it exits. The child also gets
    # its own process group.
    set -m
    "$@" &
    child=$!
    set +m
    [ -z "${pending}" ] || kill -INT "${child}" 2>/dev/null || true

    # A trap makes wait return early, so wait until the child is gone.
    while :; do
        if wait "${child}"; then status=0; else status=$?; fi
        kill -0 "${child}" 2>/dev/null || break
    done
    # A wait cut short by a trap returns 128 + the signal, so ask once more for the child's own
    # status. Bash keeps it after the child is reaped; 127 (unknown pid) keeps the loop's status.
    if wait "${child}" 2>/dev/null; then
        status=0
    else
        rc=$?
        [ "${rc}" -eq 127 ] || status=${rc}
    fi

    exit "${status}"
fi
