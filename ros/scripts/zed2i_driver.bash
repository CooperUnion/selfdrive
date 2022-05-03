#!/bin/bash

bash check_master.bash

container_name=zed

if [ ! "$(docker ps -q -f name=$container_name)" ]; then
	if [ "$(docker ps -aq -f status=exited -f name=$container_name)" ]; then
		echo "INFO: removing exited container"
		docker rm $container_name
	fi

	# start the container
	echo "INFO: starting docker compose"
	docker-compose -f ../docker-compose.yml up -d $container_name

	# verify container is running
	if [ "$(docker ps -aq -f status=exited -f name=$container_name)" ]; then
		echo "ERROR: failed to start $container_name"
		exit
	fi
fi

echo "INFO: $container_name is running"

# enables xhost for display
xhost +si:localuser:root

# run zed driver
docker exec -itd $container_name bash -c "source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash && roslaunch zed_driver zed2i.launch"
