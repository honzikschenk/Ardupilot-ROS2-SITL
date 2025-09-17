#!/usr/bin/env bash
set -e

CONTAINER_NAME="ardupilot-ros-sitl-ws-container"

docker exec -it $CONTAINER_NAME bash
