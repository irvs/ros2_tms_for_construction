#!/usr/bin/env bash
#
# ros2_tms_for_construction container entrypoint.
#
# First phase (as root):
#   - Fix ownership of named volumes (Docker creates them as root on first
#     mount; ros user cannot write otherwise).
#   - Re-exec self as the unprivileged `ros` user via gosu.
#
# Second phase (as ros):
#   - Source ROS 2 + workspace install.
#   - On first run (no success sentinel), run `colcon build --symlink-install`
#     and touch the sentinel only on success. This avoids treating a
#     half-built install/ as "already built" on the next start.
#   - exec the command (defaults to bash via Dockerfile CMD).

set -e

if [ "$(id -u)" = "0" ]; then
    for d in build install log; do
        if [ -d "/workspace/$d" ] && [ "$(stat -c '%U' "/workspace/$d")" != "ros" ]; then
            chown -R ros:ros "/workspace/$d"
        fi
    done
    exec gosu ros "$0" "$@"
fi

source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

cd /workspace

SENTINEL=/workspace/install/.colcon_build_succeeded

if [ ! -f "${SENTINEL}" ]; then
    echo "[entrypoint] Initial colcon build (no success sentinel at ${SENTINEL})"
    if colcon build --symlink-install; then
        touch "${SENTINEL}"
    else
        echo "[entrypoint] colcon build failed. Dropping into shell for debugging." >&2
        exec bash
    fi
fi

if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
fi

exec "$@"
