#!/bin/bash

# source setup environments for ros and catkin workspace
source /opt/ros/noetic/setup.bash

# https://unix.stackexchange.com/questions/466999/what-does-exec-do
exec "$@"
