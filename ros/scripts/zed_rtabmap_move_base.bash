#!/bin/bash

container_name=zed

bash check_container.bash master
bash check_container.bash zed
bash check_container.bash rtabmap
bash check_container.bash navigation

# enables xhost for display
xhost +si:localuser:root

# run custom zed launch file
docker exec -itd $container_name bash -c "source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash && roslaunch zed_driver zed2i.launch"

# run custom rtabmap launch file (car_launch/launch/rtabmap.launch)

# run custom navigation launch file (scooter_launch/launch/move_base.launch)