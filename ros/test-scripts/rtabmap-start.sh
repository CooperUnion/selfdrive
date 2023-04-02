#!/bin/bash

init() {
	tmux send-keys Enter
	tmux send-keys 'source-ros'
	tmux send-keys Enter
	tmux send-keys 'source-work'
	tmux send-keys Enter
}

sudo setenforce 0
xhost +si:localuser:root

if [ "$1" == "clean" ]; then
	tmux kill-session -t stackcli
fi

docker-compose up -d zed rtabmap master velodyne redis encoder_odom techbus

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

tmux split-window -v 'fish'

tmux rename-window 'techbus'
tmux send-keys 'docker exec -it techbus bash'
init
tmux send-keys 'rosrun src techbus.py'
tmux send-keys Enter
tmux send-keys 'sudo apt install python3.9'
tmux send-keys Enter
tmux send-keys 'python3.9 -m pip install opencan-cand'
tmux send-keys Enter

tmux split-window -h 'fish'

tmux rename-window 'encoder'
tmux send-keys 'docker exec -it encoder_odom bash'
init
tmux send-keys 'rosrun src encoder_odom.py'
tmux send-keys Enter

tmux split-window -v 'redis'

tmux rename-window 'CAN'
tmux send-keys 'sudo ip link set can0 up type can bitrate 500000'
tmux send-keys Enter
tmux send-keys 'cand --dev can0 --dbc ../../build/can/igvc_can.dbc'
tmux send-keys Enter
tmux send-keys 'candump can0 | cantools decode build/can/igvc_can.dbc -s | grep Cmd'
tmux send-keys Enter

