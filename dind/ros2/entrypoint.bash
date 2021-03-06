#!/bin/bash

echo '======== Loading the ROS 2 environment ========'
CMD='source /opt/ros/${ROS2_DISTRO}/setup.sh'
echo "$CMD" && eval "$CMD" || exit $?

if [ ! -z "$PRE_BUILD" ]; then
  echo '======== Running the pre-build command ========'
  echo "$PRE_BUILD" && eval "$PRE_BUILD" || exit $?
fi

echo '======== Building the workspace ========'
cd /ws && colcon build \
  --event-handlers console_cohesion+ \
  --cmake-args \
  --symlink-install || exit $?

if [ ! -z "$POST_BUILD" ]; then
  echo '======== Running the post-build command ========'
  echo "$POST_BUILD" && eval "$POST_BUILD" || exit $?
fi

if [ ! -z "$PRE_TEST" ]; then
  echo '======== Running the pre-test command ========'
  echo "$PRE_TEST" && eval "$PRE_TEST" || exit $?
fi

echo '======== Testing the workspace ========'
cd /ws && colcon test \
  --event-handlers console_cohesion+ \
  --pytest-with-coverage \
  --return-code-on-test-failure || exit $?

if [ ! -z "$POST_TEST" ]; then
  echo '======== Running the post-test command ========'
  echo "$POST_TEST" && eval "$POST_TEST" || exit $?
fi
