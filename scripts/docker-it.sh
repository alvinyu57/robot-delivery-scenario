#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

docker run --rm -it \
    --user "$(id -u):$(id -g)" \
    -v "$WORKSPACE_DIR:/workspace" \
    -w /workspace \
    --network host \
    --ipc host \
    --device /dev/dri \
    -e DISPLAY="${DISPLAY}" \
    -e QT_QPA_PLATFORM=xcb \
    -e QT_ENABLE_HIGHDPI_SCALING=0 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    delivery-robot:jazzy