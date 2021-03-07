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

# mkdir "/ros2/ws" && cp -r "${GITHUB_WORKSPACE}" "/ros2/ws" || exit $?

echo ''
echo '======== Building the ROS 2 image ========'
echo ''


docker build \
  --build-arg ROS2_DISTRO="${ROS2_DISTRO}" \
  -t ros2-ci:latest /ros2 || exit $?

echo ''
echo '======== Running the ROS 2 container ========'
echo ''

docker run \
  --mount type=bind,source="${GITHUB_WORKSPACE}",target=/ws/repo \
  --env PRE_BUILD="${PRE_BUILD}" \
  --env POST_BUILD="${POST_BUILD}" \
  --env PRE_TEST="${PRE_TEST}" \
  --env POST_TEST="${POST_TEST}" \
  --rm ros2-ci:latest || exit $?
