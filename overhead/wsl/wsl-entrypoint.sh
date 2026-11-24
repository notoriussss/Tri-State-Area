#!/usr/bin/env bash
# Runs inside the tsa-ros WSL2 distro.
# Sources ROS 2 Lyrical and robots.env, builds interfaces + overhead into ~/tsa-ws
# (never inside the Windows repo), sources the overlay, and runs the given command.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
ENV_FILE="$SCRIPT_DIR/robots.env"
WS_DIR="$HOME/tsa-ws"

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
set -u

if [ -f "$ENV_FILE" ]; then
    # Strip CR characters in case Windows Git checked out CRLF line endings.
    # shellcheck disable=SC1090
    source <(sed 's/\r$//' "$ENV_FILE")
fi

mkdir -p "$WS_DIR"
colcon \
    --log-base "$WS_DIR/log" \
    build \
    --symlink-install \
    --build-base "$WS_DIR/build" \
    --install-base "$WS_DIR/install" \
    --base-paths \
        "$REPO_ROOT/interfaces" \
        "$REPO_ROOT/overhead"

set +u
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"
set -u

if [ "$#" -eq 0 ]; then
    exec bash
fi

if [ "$#" -eq 1 ] && [[ "$1" == *[[:space:]]* ]]; then
    exec bash -c "$1"
fi

exec "$@"
