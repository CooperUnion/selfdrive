#!/bin/bash


if [ "$1" == "clean" ]; then
	tmux kill-session -t stackcli
fi

init() {
	tmux send-keys Enter
	tmux send-keys 'export ROS_MODE=true'
    tmux send-keys Enter
    tmux send-keys 'direnv reload'
    tmux send-keys Enter
    tmux send-keys 'source /opt/ros/humble/local_setup.bash'
    tmux send-keys Enter
}

tmux new-session -d -s stackcli

tmux select-window -t stackcli:0
tmux rename-window 'Master'
init
tmux send-keys 'python3 techbus/src/techbus.py'
tmux send-keys Enter

tmux split-window -h
init
tmux send-keys 'python3 encoder_odom/src/encoder_odom.py'
tmux send-keys Enter

tmux select-pane -t 0
tmux split-window -v
init
tmux send-keys 'rviz2' # Include rviz2 configuration file with proper settings
tmux send-keys Enter

tmux select-pane -t 2
tmux split-window -v
init
# tmux send-keys 'python3 cmd/src/main.py'
# tmux send-keys Enter

##### ADD NEW TMUX WINDOW FOR THE UI #####
# tmux new-window -n 'UI'
# tmux select-window -t 'UI'

tmux attach-session -t stackcli
