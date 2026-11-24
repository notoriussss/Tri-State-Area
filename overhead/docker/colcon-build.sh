#!/usr/bin/env bash
# Builds the interface and overhead packages of the mounted repository.
set -e

source /opt/ros/lyrical/setup.bash

cd /ws
colcon build \
    --symlink-install \
    --base-paths \
        src/tri-state-area/interfaces \
        src/tri-state-area/overhead \
    "$@"
