#!/bin/sh

ROS2_DISTRO="$1"
APT_PACKAGES="$2"
PRE_INSTALL="$3"
POST_INSTALL="$4"
PRE_BUILD="$5"
POST_BUILD="$6"
PRE_TEST="$7"
POST_TEST="$8"

echo ''
echo '======== Running the Docker daemon ========'
echo ''

dockerd-entrypoint.sh &
sleep 10

mkdir "/ros2/ws" && cp -r "${GITHUB_WORKSPACE}" "/ros2/ws" || exit $?

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
  --env APT_PACKAGES="${APT_PACKAGES}" \
  --env PRE_INSTALL="${PRE_INSTALL}" \
  --env POST_INSTALL="${POST_INSTALL}" \
  --env PRE_BUILD="${PRE_BUILD}" \
  --env POST_BUILD="${POST_BUILD}" \
  --env PRE_TEST="${PRE_TEST}" \
  --env POST_TEST="${POST_TEST}" \
  --rm ros2-ci:latest || exit $?
