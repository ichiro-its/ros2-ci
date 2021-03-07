#!/bin/sh

ROS2_DISTRO="$1"
PRE_BUILD="$2"
POST_BUILD="$3"
PRE_TEST="$4"
POST_TEST="$5"

echo ''
echo '======== Running the Docker daemon ========'
echo ''

dockerd-entrypoint.sh &
sleep 10

echo ''
echo '======== Building the ROS 2 image ========'
echo ''


docker build \
  --build-arg ROS2_DISTRO="${ROS2_DISTRO}" \
  -t ros2-ci:latest /ros2 || exit $?

echo 'running the ROS 2 container...'
docker run \
  -v "${GITHUB_WORKSPACE}":/ws/repo \
  --env PRE_BUILD="${PRE_BUILD}" \
  --env POST_BUILD="${POST_BUILD}" \
  --env PRE_TEST="${PRE_TEST}" \
  --env POST_TEST="${POST_TEST}" \
  --rm ros2-ci:latest || exit $?

echo ''
echo '======== Checking the result ========'
echo ''

cd "${GITHUB_WORKSPACE}" && ls -R .ws || exit $?
