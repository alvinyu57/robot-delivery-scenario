docker run --rm -it \
    --user "$(id -u):$(id -g)" \
    -w /workspace \
    --network host \
    --ipc host \
    --device /dev/dri \
    -e DISPLAY="${DISPLAY}" \
    -e QT_QPA_PLATFORM=xcb \
    -e QT_ENABLE_HIGHDPI_SCALING=0 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "${PWD}:/workspace" \
    delivery-robot:jazzy