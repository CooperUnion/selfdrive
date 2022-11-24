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

docker-compose up -d gazebo rtabmap master navigation

tmux new-session -d -s stackcli 'fish'

tmux select-window -t stackcli:0

tmux rename-window 'master'
tmux send-keys 'docker exec -it master bash'
init

tmux split-window -h 'fish'

tmux rename-window 'rtabmap'
tmux send-keys 'docker exec -it rtabmap bash'
init
tmux send-keys "roslaunch rtabmap_launch rtabmap_gazebo.launch $2"
tmux send-keys Enter

tmux split-window -v 'fish'

tmux rename-window 'navigation'
tmux send-keys 'docker exec -it navigation bash'
init
tmux send-keys 'roslaunch scooter_launch move_base.launch'
tmux send-keys Enter

tmux split-window -h 'fish'

tmux rename-window 'gazebo'
tmux send-keys 'docker exec -it gazebo bash'
init
tmux send-keys 'roslaunch igvc_self_drive_gazebo f1_gazebo.launch'
tmux send-keys 'cd ros_ws'
tmux send-keys 'rosdep install --from-paths src --ignore-src -r -y'
tmux send-keys 'catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release'
tmux send-keys Enter

tmux split-window -h 'fish'

tmux rename-window 'teleop'
tmux send-keys 'docker exec -it gazebo bash'
init
tmux send-keys 'rosrun teleop_twist_keyboard teleop+twist_keyboard.py'
tmux send-keys Enter