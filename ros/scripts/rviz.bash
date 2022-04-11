#!/bin/bash

container_name=rviz

bash check_container.bash master
bash check_container.bash $container_name

# enables xhost for display
xhost +si:localuser:root

# run rviz
docker exec -itd $container_name bash -c "source /opt/ros/noetic/setup.bash && rosrun rviz rviz"

# https://linuxhint.com/bash_wait_keypress/
echo "press q to kill process"
while : ; do
read -n 1 k <&1
if [[ $k = q ]] ; then
printf "\nQuitting from the program\n"
docker exec -itd $container_name bash -c "pkill rosrun"
break
else
echo "Press 'q' to exit"
fi
done