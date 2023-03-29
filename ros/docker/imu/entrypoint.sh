#!/bin/bash

# if any command fails, exit script
set -e

# source setup environments for ros and catkin workspace
source /opt/ros/noetic/setup.bash

# cd ros_ws/src/imu/kvh1750_ws/
# catkin_make
# source devel/setup.bash

# https://unix.stackexchange.com/questions/466999/what-does-exec-do
exec "$@"