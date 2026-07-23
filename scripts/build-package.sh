#!/bin/bash

set -euo pipefail

ORIGINAL_ARGS=("$@")
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

show_usage() {
    echo "Usage: $0 [--test] [--docker]"
    echo
    echo "Builds the ROS2 package."
    echo "  --test, test        Run package tests after building."
    echo "  --docker, docker    Build package in Docker."
    echo "  -h, --help          Show this help message."
}

run_tests=false
run_in_docker=false

while [[ $# -gt 0 ]]; do
    case "$1" in
    "--test"|"test")
        run_tests=true
        shift
        ;;
    "--docker"|"docker")
        run_in_docker=true
        shift
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
done

if [ "$run_in_docker" = true ]; then
    echo "Building in Docker..."

    command_name=$(basename "$0")
    args=()
    for arg in "${ORIGINAL_ARGS[@]}"; do
        if [[ "$arg" != "--docker" && "$arg" != "docker" ]]; then
            args+=("$arg")
        fi
    done

    inner_command="./scripts/$command_name"
    for arg in "${args[@]}"; do
        inner_command+=" $(printf '%q' "$arg")"
    done

    docker_tty_args=()
    if [ -t 0 ]; then
        docker_tty_args=(-it)
    fi

    docker run --rm "${docker_tty_args[@]}" \
        --user "$(id -u):$(id -g)" \
        -v "${WORKSPACE_DIR}:/workspace" \
        -w /workspace \
        delivery-robot:jazzy \
        bash -c "$inner_command"

    exit 0
fi

if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "ROS distro 'jazzy' is not installed at /opt/ros/jazzy."
    echo "Set ROS_DISTRO to an installed distro or build the Docker image first."
    exit 1
fi

cd "$WORKSPACE_DIR"

set +u
source "/opt/ros/jazzy/setup.bash"
set -u

rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy

colcon build

if [ "$run_tests" = true ]; then
    colcon test
    colcon test-result --verbose
fi