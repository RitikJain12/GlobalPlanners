#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
echo "ROS 2 Humble environment sourced successfully."
exec "$@"