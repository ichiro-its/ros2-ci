#!/bin/bash

echo ''
echo '======== Loading the ROS 2 environment ========'
echo ''

CMD="source /opt/ros/${ROS2_DISTRO}/setup.sh"
echo "$CMD" && eval "$CMD" || exit $?

if [ ! -z "$PRE_INSTALL" ]
then
  echo ''
  echo '======== Running the pre-install command ========'
  echo ''

  cd /ws/repo && echo "$PRE_INSTALL" && eval "$PRE_INSTALL" || exit $?
fi

if [ ! -z "$APT_PACKAGES" ]
then
  echo ''
  echo '======== Installing APT packages ========'
  echo ''

  echo "$APT_PACKAGES" && apt-get update && apt-get install -y $APT_PACKAGES || exit $?
fi

if [ ! -z "$PIP_PACKAGES" ]
then
  echo ''
  echo '======== Installing pip packages ========'
  echo ''

  echo "$PIP_PACKAGES" && pip3 install $PIP_PACKAGES || exit $?
fi

if [ ! -z "$EXTERNAL_REPOS" ]
then
  echo ''
  echo '======== Cloning external repositories ========'
  echo ''

  cd /ws || exit $?

  for REPO in $EXTERNAL_REPOS
  do
    VALUES=(${REPO//#/ })

    URL="${VALUES[0]}"
    BRANCH="${VALUES[1]}"

    if [ -z "$BRANCH" ]
    then
      CMD="git clone $URL"
      if ! (echo "$CMD" && eval "$CMD")
      then
        CMD="git clone https://github.com/$URL.git"
        echo "$CMD" && eval "$CMD" || exit $?
      fi
    else
      CMD="git clone -b $BRANCH $URL"
      if ! (echo "$CMD" && eval "$CMD")
      then
        CMD="git clone -b $BRANCH https://github.com/$URL.git"
        echo "$CMD" && eval "$CMD" || exit $?
      fi
    fi
  done
fi

if [ ! -z "$POST_INSTALL" ]
then
  echo ''
  echo '======== Running the post-install command ========'
  echo ''

  cd /ws/repo && echo "$POST_INSTALL" && eval "$POST_INSTALL" || exit $?
fi

if [ ! -z "$PRE_BUILD" ]
then
  echo ''
  echo '======== Running the pre-build command ========'
  echo ''

  cd /ws/repo && echo "$PRE_BUILD" && eval "$PRE_BUILD" || exit $?
fi

if [ "$REPOS_FILEPATH" = "" ]; then
  echo ''
  echo '======== Skipping install packages via repos file ========'
  echo ''
else
  echo ''
  echo '======== Install packages via repos file ========'
  echo ''
  ls /ws
  ls /ws/src
  cat /ws/src/"$REPOS_FILEPATH"
  cd /ws && vcs import src < /ws/src/"$REPOS_FILEPATH"
fi


echo ''
echo '======== Building the workspace ========'
echo ''

cd /ws && colcon build \
  --event-handlers console_cohesion+ \
  --cmake-args || exit $?

source install/setup.bash || exit $?

if [ ! -z "$POST_BUILD" ]
then
  echo ''
  echo '======== Running the post-build command ========'
  echo ''

  cd /ws/repo && echo "$POST_BUILD" && eval "$POST_BUILD" || exit $?
fi

if [ ! -z "$PRE_TEST" ]
then
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

mkdir /ws/repo/.ws \
  && cp -r /ws/build /ws/repo/.ws \
  && cp -r /ws/log /ws/repo/.ws \
  && cp -r /ws/install /ws/repo/.ws \
  || exit $?

if [ ! -z "$POST_TEST" ]
then
  echo ''
  echo '======== Running the post-test command ========'
  echo ''

  cd /ws/repo && echo "$POST_TEST" && eval "$POST_TEST" || exit $?
fi
