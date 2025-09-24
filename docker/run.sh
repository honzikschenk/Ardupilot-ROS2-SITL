#!/usr/bin/env bash
set -e

IMAGE_NAME="ardupilot-ros-sitl-ws"
CONTAINER_NAME="ardupilot-ros-sitl-ws-container"

# Base Docker arguments
DOCKER_ARGS=(
    -it
    --name "$CONTAINER_NAME"
    --rm
    --ipc host
    --pid host
    --env QT_X11_NO_MITSHM=1
    --env DISPLAY
    --network host
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw"
    --volume "$(pwd):/workspace"
)

# Allow local docker clients to access X (ignore errors if xhost absent)
xhost +local:docker > /dev/null 2>&1 || true

# Remove old container if it exists
if docker ps -aq -f name=^${CONTAINER_NAME}$ > /dev/null 2>&1; then
    if [[ -n "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]]; then
        docker rm -f "$CONTAINER_NAME" > /dev/null 2>&1 || true
    fi
fi

docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash
