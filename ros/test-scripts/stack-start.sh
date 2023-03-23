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

# docker compose down --remove-orphans

echo "redis-cli shutdown"

docker compose up -d master zed rtabmap velodyne navigation encoder_odom techbus redis pure_pursuit

tmux new-session -d -s stackcli 'fish'

tmux select-window -t stackcli:0
tmux rename-window 'Master'
tmux send-keys 'docker exec -it master bash'
init

tmux new-window -n 'CAN'
tmux select-window -t 'CAN'
tmux send-keys 'docker exec -it techbus bash'
init
# tmux send-keys 'rosrun src techbus.py'
# tmux send-keys Enter

tmux split-window -h 'fish'
tmux send-keys 'cand --dbc build/can/igvc_can.dbc --dev vcan0'
tmux send-keys Enter

tmux select-pane -t 0
tmux split-window -v 'fish'
# tmux send-keys 'sudo ip link set can0 up type can bitrate 500000'
# tmux send-keys Enter
# tmux send-keys 'cand --dev can0 --dbc ../../build/can/igvc_can.dbc'
# tmux send-keys Enter
# tmux send-keys 'candump can0 | cantools decode build/can/igvc_can.dbc -s | grep Cmd'
# tmux send-keys Enter

tmux new-window -n 'Perception'
tmux select-window -t 'Perception'
tmux send-keys 'docker exec -it zed bash'
init
# tmux send-keys 'roslaunch zed_launch zed_no_tf.launch'
# tmux send-keys Enter

tmux split-window -h 'fish'
tmux send-keys 'docker exec -it velodyne bash'
init
# tmux send-keys 'roslaunch velodyne_pointcloud VLP16_points.launch'
# tmux send-keys Enter 

tmux select-pane -t 0
tmux split-window -v 'fish'
tmux send-keys 'docker exec -it encoder_odom bash'
init
# tmux send-keys 'rosrun src encoder_odom.py'
# tmux send-keys Enter

tmux split-window -h 'fish'
tmux send-keys 'docker exec -it rviz bash'
init
# tmux send-keys 'rviz'
# tmux send-keys Enter 

tmux new-window -n 'Navigation'
tmux select-window -t 'Navigation'
tmux send-keys 'docker exec -it rtabmap bash'
init
# tmux send-keys "roslaunch rtabmap_launch rtabmap.launch $2"
# tmux send-keys Enter

tmux split-window -h 'fish'
tmux send-keys 'docker exec -it navigation bash'
init
# tmux send-keys 'roslaunch scooter_launch move_base.launch'
# tmux send-keys Enter

tmux select-pane -t 0
tmux split-window -v 'fish'
tmux send-keys 'docker exec -it pure_pursuit bash'
init
# tmux send-keys 'roslaunch pure_pursuit pure_pursuit.launch'
# tmux send-keys Enter

tmux attach-session -t stackcli