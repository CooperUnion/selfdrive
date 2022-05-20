#!/bin/bash

bash check_container.bash master
bash check_container.bash zed
bash check_container.bash rtabmap
bash check_container.bash navigation

# enables xhost for display
xhost +si:localuser:root

# run custom zed launch file
docker exec -itd zed bash -c "source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash && roslaunch car_launch zed2i.launch"

# run custom rtabmap launch file (car_launch/launch/rtabmap.launch)
docker exec -itd rtabmap bash -c "source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash && roslaunch car_launch rtabmap.launch"

# run custom navigation launch file (scooter_launch/launch/move_base.launch)
docker exec -itd navigation bash -c "source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash && roslaunch scooter_launch move_base.launch"

echo "press q to kill process"
while : ; do
read -n 1 k <&1
if [[ $k = q ]] ; then
printf "\nQuitting from the program\n"
docker exec -itd zed bash -c "pkill roslaunch"
docker exec -itd rtabmap bash -c "pkill roslaunch"
docker exec -itd navigation bash -c "pkill roslaunch"
break
else
echo "Press 'q' to exit"
fi
done