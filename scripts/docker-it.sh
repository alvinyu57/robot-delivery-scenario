#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

show_usage() {
    echo "Usage: $0 [--gui | --headless]"
    echo
    echo "Starts an interactive development container."
    echo "  --gui       Enable Gazebo and RViz through the host X11 display (default)."
    echo "  --headless  Disable Gazebo and RViz GUIs; no display or GPU is required."
    echo "  -h, --help  Show this help message."
}

mode="gui"
if [[ $# -gt 1 ]]; then
    show_usage
    exit 1
fi

if [[ $# -eq 1 ]]; then
    case "$1" in
    "--gui"|"gui")
        mode="gui"
        ;;
    "--headless"|"headless")
        mode="headless"
        ;;
    "-h"|"--help")
        show_usage
        exit 0
        ;;
    *)
        show_usage
        exit 1
        ;;
    esac
fi

docker_args=(
    run
    --rm
    -it
    --user "$(id -u):$(id -g)"
    -v "$WORKSPACE_DIR:/workspace"
    -w /workspace
    --network host
    --ipc host
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
)

if [[ "$mode" == "gui" ]]; then
    if [[ -z "${DISPLAY:-}" ]]; then
        echo "DISPLAY is not set. Use --headless or provide an X11 display."
        exit 1
    fi

    if [[ ! -e /dev/dri ]]; then
        echo "/dev/dri is not available. Use --headless or configure GPU access."
        exit 1
    fi

    if [[ ! -d /tmp/.X11-unix ]]; then
        echo "/tmp/.X11-unix is not available. Use --headless or configure X11."
        exit 1
    fi

    xauthority_file="${XAUTHORITY:-}"
    if [[ -z "$xauthority_file" && -r "$HOME/.Xauthority" ]]; then
        xauthority_file="$HOME/.Xauthority"
    fi
    if [[ -z "$xauthority_file" || ! -r "$xauthority_file" ]]; then
        echo "No readable X11 authorization file was found."
        echo "Set XAUTHORITY to the host X11/Xwayland cookie or use --headless."
        exit 1
    fi

    declare -A added_device_groups=()
    for device_path in /dev/dri/card* /dev/dri/renderD*; do
        if [[ ! -e "$device_path" ]]; then
            continue
        fi
        device_group="$(stat -c '%g' "$device_path")"
        if [[ -z "${added_device_groups[$device_group]+set}" ]]; then
            docker_args+=(--group-add "$device_group")
            added_device_groups[$device_group]=1
        fi
    done

    docker_args+=(
        --device /dev/dri
        -e "DISPLAY=$DISPLAY"
        -e XAUTHORITY=/tmp/.docker.xauthority
        -e GUI=true
        -e QT_QPA_PLATFORM=xcb
        -e QT_ENABLE_HIGHDPI_SCALING=0
        -v "$xauthority_file:/tmp/.docker.xauthority:ro"
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    )
else
    docker_args+=(-e GUI=false)
fi

exec docker "${docker_args[@]}" delivery-robot:jazzy
