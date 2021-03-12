#!/bin/sh

ROS2_DISTRO="$1"
APT_PACKAGES="$2"
PIP_PACKAGES="$3"
PRE_INSTALL="$4"
POST_INSTALL="$5"
PRE_BUILD="$6"
POST_BUILD="$7"
PRE_TEST="$8"
POST_TEST="$9"

echo ''
echo '======== Running the Docker daemon ========'
echo ''

dockerd-entrypoint.sh &
sleep 10

mkdir "/ros2/ws" && cp -r "${GITHUB_WORKSPACE}" "/ros2/ws/repo" || exit $?

echo ''
echo '======== Building the ROS 2 image ========'
echo ''


docker build \
  --build-arg ROS2_DISTRO="${ROS2_DISTRO}" \
  -t ros2-ci:latest /ros2 || exit $?

echo 'Copying the repository files...'
mkdir -p /ws && cp -r "${GITHUB_WORKSPACE}" /ws/repo || exit $?

echo 'Running the ROS 2 container...'
docker run \
  -v /ws/repo:/ws/repo \
  --env APT_PACKAGES="${APT_PACKAGES}" \
  --env PIP_PACKAGES="${PIP_PACKAGES}" \
  --env PRE_INSTALL="${PRE_INSTALL}" \
  --env POST_INSTALL="${POST_INSTALL}" \
  --env PRE_BUILD="${PRE_BUILD}" \
  --env POST_BUILD="${POST_BUILD}" \
  --env PRE_TEST="${PRE_TEST}" \
  --env POST_TEST="${POST_TEST}" \
  --rm ros2-ci:latest || exit $?

echo ''
echo '======== Checking the result ========'
echo ''

echo 'Copying the result files...'
cp -r /ws/repo/.ws "${GITHUB_WORKSPACE}" || exit $?
cd "${GITHUB_WORKSPACE}" && ls && ls -R .ws || exit $?
