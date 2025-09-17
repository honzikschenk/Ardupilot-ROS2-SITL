#!/usr/bin/env bash
set -e

IMAGE_NAME="ardupilot-ros-sitl-ws"

docker build -t $IMAGE_NAME -f docker/Dockerfile .
