#!/bin/bash

cp -r "${GITHUB_REPOSITORY}" "/ros/${GITHUB_WORKSPACE}"

docker build \
  --build-args ROS2_DISTRO="$1" \
  -t ros2-ci:latest /ros2 || exit $?

docker run --rm ros2-ci:latest || exit $?
