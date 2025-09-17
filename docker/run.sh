#!/usr/bin/env bash
set -e

IMAGE_NAME="ardupilot-ros-sitl-ws"
CONTAINER_NAME="ardupilot-ros-sitl-ws-container"

OS_TYPE="$(uname -s)"

# Default Docker args common to both
DOCKER_ARGS=(
    -it
    --name "$CONTAINER_NAME"
    --rm
    --ipc host
    --pid host
    --env QT_X11_NO_MITSHM=1
    --volume "$(pwd):/workspace"
)

if [[ "$OS_TYPE" == "Darwin" ]]; then
    # macOS: use XQuartz over TCP. Always force DISPLAY to host.docker.internal:0 to avoid launchd paths.
    # Ensure XQuartz is running and "Allow connections from network clients" is enabled.
    export DISPLAY="host.docker.internal:0"

    # Prepare Xauthority for XQuartz to avoid "Authorization required" and xcb plugin errors.
    # XQuartz stores MIT-MAGIC-COOKIEs in ~/.Xauthority when network clients are allowed.
    XAUTH_FILE="$(mktemp -t .docker.xauth.XXXXXXXX)"
    if command -v xauth >/dev/null 2>&1; then
        # Try several common forms that XQuartz uses for local display cookies
        LOCAL_DSP=":${DISPLAY##*:}"
        {
            xauth nlist "$LOCAL_DSP" 2>/dev/null || true;
            xauth nlist "$HOSTNAME/unix${LOCAL_DSP}" 2>/dev/null || true;
            xauth nlist "$HOST/unix${LOCAL_DSP}" 2>/dev/null || true;
            xauth nlist "$USER@${HOSTNAME}/unix${LOCAL_DSP}" 2>/dev/null || true;
        } | sed -e 's/^..../ffff/' | xauth -f "$XAUTH_FILE" nmerge - >/dev/null 2>&1 || true
        if [ ! -s "$XAUTH_FILE" ]; then
            # If still empty, try merging default ~/.Xauthority if present
            [ -f "$HOME/.Xauthority" ] && xauth -f "$XAUTH_FILE" nmerge "$HOME/.Xauthority" >/dev/null 2>&1 || true
        fi
    else
        # xauth may not be installed on the mac host; GUI may still work if XQuartz allows unauthenticated connections
        XAUTH_FILE=""
    fi
    DOCKER_ARGS+=(
        --env DISPLAY="$DISPLAY"
        --env LIBGL_ALWAYS_INDIRECT=1
        --env QT_QPA_PLATFORM=xcb
        --env XAUTHORITY=/tmp/.docker_xauth
    )
    # Mount Xauthority if created
    if [ -n "$XAUTH_FILE" ] && [ -s "$XAUTH_FILE" ]; then
        DOCKER_ARGS+=( --volume "$XAUTH_FILE:/tmp/.docker_xauth:ro" )
    fi
else
    # Linux: map the X11 socket and use host networking
    xhost +local:docker > /dev/null 2>&1 || true
    DOCKER_ARGS+=(
        --network host
        --env DISPLAY
        --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw"
    )
fi

# Remove old container if it exists
if [ "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]; then
        docker rm -f "$CONTAINER_NAME" > /dev/null 2>&1 || true
fi

docker run "${DOCKER_ARGS[@]}" "$IMAGE_NAME" bash
