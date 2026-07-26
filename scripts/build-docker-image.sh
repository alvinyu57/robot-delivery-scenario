#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

docker build \
    --pull \
    -t delivery-robot:jazzy \
    --build-arg UID="$(id -u)" \
    --build-arg GID="$(id -g)" \
    --build-arg USER="$(whoami)" \
    "$WORKSPACE_DIR"
