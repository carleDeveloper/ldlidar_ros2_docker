#!/usr/bin/env bash
set -e

# Source ROS 2 and the built workspace overlay so ros2 commands work.
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

exec "$@"
