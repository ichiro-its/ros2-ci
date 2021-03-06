#!/bin/bash

echo '======== Loading the ROS 2 environment ========'
source /opt/ros/${ROS2_DISTRO}/setup.sh || exit $?

if [ ! -z "$PRE_BUILD" ]; then
  echo '======== Running the pre-build command ========'
  eval "$PRE_BUILD" || exit $?
fi

echo '======== Building the workspace ========'
cd /ws && colcon build \
  --event-handlers console_cohesion+ \
  --cmake-args \
  --symlink-install || exit $?

if [ ! -z "$POST_BUILD" ]; then
  echo '======== Running the post-build command ========'
  eval "$POST_BUILD" || exit $?
fi

if [ ! -z "$PRE_TEST" ]; then
  echo '======== Running the pre-test command ========'
  eval "$PRE_TEST" || exit $?
fi

echo '======== Testing the workspace ========'
cd /ws && colcon test \
  --event-handlers console_cohesion+ \
  --pytest-with-coverage \
  --return-code-on-test-failure || exit $?

if [ ! -z "$POST_TEST" ]; then
  echo '======== Running the post-test command ========'
  eval "$POST_TEST" || exit $?
fi
