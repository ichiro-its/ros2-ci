#!/bin/bash

echo '\n======== Loading the ROS 2 environment ========\n'
CMD="source /opt/ros/${ROS2_DISTRO}/setup.sh"
echo "$CMD" && eval "$CMD" || exit $?

if [ ! -z "$PRE_BUILD" ]; then
  echo '\n======== Running the pre-build command ========\n'
  echo "$PRE_BUILD" && eval "$PRE_BUILD" || exit $?
fi

echo '\n======== Building the workspace ========\n'
cd /ws && colcon build \
  --event-handlers console_cohesion+ \
  --cmake-args \
  --symlink-install || exit $?

if [ ! -z "$POST_BUILD" ]; then
  echo '\n======== Running the post-build command ========\n'
  echo "$POST_BUILD" && eval "$POST_BUILD" || exit $?
fi

if [ ! -z "$PRE_TEST" ]; then
  echo '\n======== Running the pre-test command ========\n'
  echo "$PRE_TEST" && eval "$PRE_TEST" || exit $?
fi

echo '\n======== Testing the workspace ========\n'
cd /ws && colcon test \
  --event-handlers console_cohesion+ \
  --pytest-with-coverage \
  --return-code-on-test-failure || exit $?

if [ ! -z "$POST_TEST" ]; then
  echo '\n======== Running the post-test command ========\n'
  echo "$POST_TEST" && eval "$POST_TEST" || exit $?
fi
