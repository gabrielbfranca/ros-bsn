#!/bin/bash
# Sourcing ROS2 and workspace setup files
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash

# Execute the command passed to the container (if any)
exec "$@"
