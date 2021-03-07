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

echo 'Copying the repository files...'
cp -r "${GITHUB_WORKSPACE}" /ws/repo || exit $?

echo 'Running the ROS 2 container...'
docker run \
  -v /ws/repo:/ws/repo \
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
