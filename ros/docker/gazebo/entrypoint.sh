#!/bin/bash

# if any command fails, exit script
set -e

# source setup environments for ros and catkin workspace
source /opt/ros/noetic/setup.bash
source /app/ros_ws/devel/setup.bash

# https://unix.stackexchange.com/questions/466999/what-does-exec-do
exec "$@"