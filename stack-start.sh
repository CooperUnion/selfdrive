#!/bin/bash

init() {
	tmux send-keys Enter
	tmux send-keys 'source-ros'
	tmux send-keys Enter
	tmux send-keys 'source-work'
	tmux send-keys Enter
}

xhost +si:localuser:root

if [ "$1" == "clean" ]; then
	tmux kill-session -t stackcli
fi

docker-compose up -d zed rtabmap master velodyne navigation gps

tmux new-session -d -s stackcli 'fish'

tmux select-window -t stackcli:0

tmux rename-window 'master'
tmux send-keys 'docker exec -it master bash'
init

tmux split-window -h 'fish'

tmux rename-window 'rtabmap'
tmux send-keys 'docker exec -it rtabmap bash'
init
tmux send-keys "roslaunch rtabmap_launch rtabmap.launch $2"
tmux send-keys Enter

tmux split-window -v 'fish'

tmux rename-window 'navigation'
tmux send-keys 'docker exec -it navigation bash'
init
tmux send-keys 'roslaunch scooter_launch move_base.launch'
tmux send-keys Enter

#tmux select-window -t 'master'

tmux split-window -v 'fish'

tmux rename-window 'zed'
tmux send-keys 'docker exec -it zed bash'
init
tmux send-keys 'roslaunch zed_launch zed_no_tf.launch'
tmux send-keys Enter

tmux split-window -h 'fish'

tmux rename-window 'velodyne'
tmux send-keys 'docker exec -it velodyne bash'
init
tmux send-keys 'roslaunch velodyne_pointcloud VLP16_points.launch'
tmux send-keys Enter

tmux split-window -h 'fish'

tmux rename-window 'gps'
tmux send-keys 'docker exec -it gps bash'
init
tmux send-keys 'echo hello'
tmux send-keys Enter



