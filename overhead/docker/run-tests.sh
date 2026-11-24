#!/usr/bin/env bash
# Builds the workspace and runs overhead core and ROS integration tests.
set -e

source /opt/ros/lyrical/setup.bash
colcon-build
source /ws/install/setup.bash

python3 -m pytest \
    /ws/src/tri-state-area/overhead/overhead_core \
    /ws/src/tri-state-area/overhead/overhead_ros/test \
    -q \
    -p no:cacheprovider \
    "$@"
