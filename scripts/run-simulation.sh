#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
GUI_VALUE="${GUI:-true}"

if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "ROS 2 Jazzy is not available in this shell."
    echo "Run ./scripts/docker-it.sh, then execute this script in the container."
    exit 1
fi

if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "The workspace has not been built."
    echo "Run ./scripts/build-package.sh first."
    exit 1
fi

set +u
source "/opt/ros/jazzy/setup.bash"
source "$WORKSPACE_DIR/install/setup.bash"
set -u

exec ros2 launch delivery_robot_description simulation.launch.py gui:="$GUI_VALUE"
