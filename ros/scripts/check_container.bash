#!/bin/bash

container_name=$1

# https://stackoverflow.com/questions/38576337/how-to-execute-a-bash-command-only-if-a-docker-container-with-a-given-name-does
# check if container is already running
if [ ! "$(docker ps -q -f name=$container_name)" ]; then
	# check if the container has exited
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
