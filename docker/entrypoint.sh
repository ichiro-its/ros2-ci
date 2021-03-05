#!/bin/bash

docker build \
  --build-args ROS2_DISTRO="$1" \
  --build-args GITHUB_WORKSPACE="$GITHUB_WORKSPACE" \
  --build-args GITHUB_REPOSITORY="$GITHUB_REPOSITORY" \
  -t ros2-ci:latest /ros2 || exit $?

docker run --rm ros2-ci:latest || exit $?
