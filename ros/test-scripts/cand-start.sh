!#/bin/bash

if [ "$1" == "clean" ]; then
	tmux kill-session -t stackcli
fi

docker-compose up -d redis

tmux new-session -d -s stackcli 'fish'

tmux select-window -t stackcli:0

tmux split-window -h 'redis'

tmux rename-window 'cand'
tmux send-keys 'cand --dev can0 --dbc ../../build/can/igvc_can.dbc'
tmux send-keys Enter

tmux split-window -v 'candump'

tmux send-keys 'sudo ip link set can0 up type can bitrate 500000'
tmux send-keys Enter
tmux send-keys 'candump can0 | cantools decode build/can/igvc_can.dbc -s | grep Cmd'
tmux send-keys Enter