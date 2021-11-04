#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /app/ros_ws/devel/setup.bash

exec "$@"