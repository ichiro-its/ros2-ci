#!/bin/bash

cd "$GITHUB_WORKSPACE" && cd .. || exit $?

source /opt/ros/foxy/setup.sh || exit $?

colcon build --event-handlers console_cohesion+ --cmake-args --symlink-install || exit $?

colcon test --event-handlers console_cohesion+ --pytest-with-coverage --return-code-on-test-failure || exit $?
