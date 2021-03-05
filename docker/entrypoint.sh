#!/bin/sh

dockerd-entrypoint.sh &
sleep 10

mkdir "/ros2/ws" && cp -r "${GITHUB_WORKSPACE}" "/ros2/ws" || exit $?

docker build \
  --build-arg ROS2_DISTRO="$1" \
  -t ros2-ci:latest /ros2 || exit $?

docker run --rm ros2-ci:latest || exit $?
