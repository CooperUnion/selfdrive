#!/bin/bash

sudo setenforce 0
xhost +si:localuser:root

if [ "$1" == "clean" ]; then
	sudo docker compose down --remove-orphans
	tmux kill-session -t stackcli
fi

echo "redis-cli shutdown"

# sudo docker compose up -d master zed rtabmap velodyne navigation encoder_odom techbus redis
sudo docker compose up -d master zed rtabmap velodyne rviz
# Remove sudo for tech computer

tmux new-session -d -s stackcli 'fish'

tmux select-window -t stackcli:0
tmux rename-window 'Master'
tmux send-keys 'sudo docker exec -it master bash'
tmux send-keys Enter

tmux new-window -n 'Perception'
tmux select-window -t 'Perception'
tmux send-keys 'sudo docker exec -it zed bash'
tmux send-keys Enter
# tmux send-keys 'roslaunch zed_launch zed_no_tf.launch'
# tmux send-keys Enter

tmux split-window -h 'fish'
tmux send-keys 'sudo docker exec -it velodyne bash'
tmux send-keys Enter
# tmux send-keys 'roslaunch velodyne_pointcloud VLP16_points.launch'
# tmux send-keys Enter 

tmux select-pane -t 0
tmux split-window -v 'fish'
tmux send-keys 'sudo docker exec -it rtabmap bash'
tmux send-keys Enter
# tmux send-keys "roslaunch rtabmap_launch ekf.launch"
# tmux send-keys Enter

tmux select-pane -t 2
tmux split-window -v 'fish'
tmux send-keys 'sudo docker exec -it rviz bash'
tmux send-keys Enter
# tmux send-keys 'rviz'
# tmux send-keys Enter 

tmux attach-session -t stackcli
