#!/usr/bin/env bash
set -e

IMAGE_NAME="ros-ws"
CONTAINER_NAME="ros-container"

if [ -z "${DISPLAY:-}" ]; then
    if getent hosts host.docker.internal >/dev/null 2>&1; then
        export DISPLAY=host.docker.internal:0
    else
        export DISPLAY=:0
    fi
fi

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

if docker ps -aq -f name=^${CONTAINER_NAME}$ > /dev/null 2>&1; then
    if [[ -n "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]]; then
        docker rm -f "$CONTAINER_NAME" > /dev/null 2>&1 || true
    fi
fi

docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash -lc "exec bash"
