#!/bin/bash

container_name=zed

bash check_container.bash master
bash check_container.bash $container_name

# enables xhost for display
xhost +si:localuser:root

# run zed driver
docker exec -itd $container_name bash -c "source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash && roslaunch zed_driver zed2i.launch"

echo "press q to kill process"
while : ; do
read -n 1 k <&1
if [[ $k = q ]] ; then
printf "\nQuitting from the program\n"
docker exec -itd $container_name bash -c "pkill roslaunch"
break
else
echo "Press 'q' to exit"
fi
done