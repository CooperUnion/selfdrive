#!/bin/bash
if [ -n "$1" ]; then
	name="$1"
else
	name="contains-carrie"
fi

docker rm "$name"
docker run \
	-it --net=host --device=/dev/ttyACM0 -v $PWD/carrie:/carrie \
	--name "$name" carrie/ros
# XXX: allow switching serial devices

# to attach again:
# docker start -i -a "$name"
