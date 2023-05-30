#!/bin/bash

# source setup environments for ros and catkin workspace
source /opt/ros/noetic/setup.bash
# source /app/ros_ws/devel/setup.bash

cd ros_ws/src/rviz/rviz_python_bindings/test
# https://unix.stackexchange.com/questions/466999/what-does-exec-do
exec "$@"