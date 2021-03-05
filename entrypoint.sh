#!/bin/bash

source /opt/ros/foxy/setup.sh && mkdir /ws && cd /ws || exit $?

mv "$GITHUB_WORKSPACE" . || $?

colcon build --event-handlers console_cohesion+ --cmake-args --symlink-install || exit $?

colcon test --event-handlers console_cohesion+ --pytest-with-coverage --return-code-on-test-failure || exit $?
