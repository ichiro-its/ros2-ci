#!/bin/bash

echo ''
echo '======== Loading the ROS 2 environment ========'
echo ''

CMD="source /opt/ros/${ROS2_DISTRO}/setup.sh"
echo "$CMD" && eval "$CMD" || exit $?

if [ ! -z "$PRE_BUILD" ]; then
  echo ''
  echo '======== Running the pre-build command ========'
  echo ''

  echo "$PRE_BUILD" && eval "$PRE_BUILD" || exit $?
fi

echo ''
echo '======== Building the workspace ========'
echo ''

cd /ws && colcon build \
  --event-handlers console_cohesion+ \
  --cmake-args || exit $?

echo ''
echo '======== Copying the build result ========'
echo ''

mkdir -p /ws/repo/.ws
cp -r /ws/build /ws/repo/.ws
cp -r /ws/log /ws/repo/.ws
cp -r /ws/install /ws/repo/.ws

if [ ! -z "$POST_BUILD" ]; then
  echo ''
  echo '======== Running the post-build command ========'
  echo ''

  echo "$POST_BUILD" && eval "$POST_BUILD" || exit $?
fi

if [ ! -z "$PRE_TEST" ]; then
  echo ''
  echo '======== Running the pre-test command ========'
  echo ''

  echo "$PRE_TEST" && eval "$PRE_TEST" || exit $?
fi

echo ''
echo '======== Testing the workspace ========'
echo ''

cd /ws && colcon test \
  --event-handlers console_cohesion+ \
  --pytest-with-coverage \
  --return-code-on-test-failure || exit $?

if [ ! -z "$POST_TEST" ]; then
  echo ''
  echo '======== Running the post-test command ========'
  echo ''

  echo "$POST_TEST" && eval "$POST_TEST" || exit $?
fi
