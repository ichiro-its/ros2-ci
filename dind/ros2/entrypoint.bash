#!/bin/bash

cd /ws/repo && ls -R || exit $?

echo "Loading the ROS 2 ${ROS2_DISTRO} environment..."
eval "source /opt/ros/${ROS2_DISTRO}/setup.sh" || exit $?

if [ ! -z "$PRE_INSTALL" ]; then
  echo ''
  echo '======== Running the pre-install command ========'
  echo ''

  cd /ws/repo && echo "$PRE_INSTALL" && eval "$PRE_INSTALL" || exit $?
fi

if [ ! -z "$APT_PACKAGES" ]; then
  echo ''
  echo '======== Installing APT packages ========'
  echo ''

  echo "$APT_PACKAGES" && apt-get update && apt-get install -y $APT_PACKAGES || exit $?
fi

if [ ! -z "$PIP_PACKAGES" ]; then
  echo ''
  echo '======== Installing pip packages ========'
  echo ''

  echo "$PIP_PACKAGES" && pip3 install $PIP_PACKAGES || exit $?
fi

if [ ! -z "$POST_INSTALL" ]; then
  echo ''
  echo '======== Running the post-install command ========'
  echo ''

  cd /ws/repo && echo "$POST_INSTALL" && eval "$POST_INSTALL" || exit $?
fi

if [ ! -z "$PRE_BUILD" ]; then
  echo ''
  echo '======== Running the pre-build command ========'
  echo ''

  cd /ws/repo && echo "$PRE_BUILD" && eval "$PRE_BUILD" || exit $?
fi

echo ''
echo '======== Building the workspace ========'
echo ''

cd /ws && colcon build \
  --event-handlers console_cohesion+ \
  --cmake-args || exit $?

mkdir /ws/repo/.ws \
  && cp -r /ws/build /ws/repo/.ws \
  && cp -r /ws/log /ws/repo/.ws \
  && cp -r /ws/install /ws/repo/.ws \
  || exit $?

if [ ! -z "$POST_BUILD" ]; then
  echo ''
  echo '======== Running the post-build command ========'
  echo ''

  cd /ws/repo && echo "$POST_BUILD" && eval "$POST_BUILD" || exit $?
fi

if [ ! -z "$PRE_TEST" ]; then
  echo ''
  echo '======== Running the pre-test command ========'
  echo ''

  cd /ws/repo && echo "$PRE_TEST" && eval "$PRE_TEST" || exit $?
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

  cd /ws/repo && echo "$POST_TEST" && eval "$POST_TEST" || exit $?
fi

echo 'Copying the build result...'
mkdir -p /ws/repo/.ws || exit $?
cp -r /ws/build /ws/repo/.ws || exit $?
cp -r /ws/log /ws/repo/.ws || exit $?
cp -r /ws/install /ws/repo/.ws || exit $?
