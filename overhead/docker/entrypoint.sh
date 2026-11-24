#!/usr/bin/env bash
# Sources ROS 2 and, when present, the overhead workspace overlay before running the command.
set -e

source /opt/ros/lyrical/setup.bash

if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

exec "$@"
