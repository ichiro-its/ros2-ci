#!/bin/bash

source /opt/ros/${ROS2_DISTRO}/setup.sh || exit $?

cd /ws && colcon build \
  --event-handlers console_cohesion+ \
  --cmake-args \
  --symlink-install || exit $?

cd /ws && colcon test \
  --event-handlers console_cohesion+ \
  --pytest-with-coverage \
  --return-code-on-test-failure || exit $?
