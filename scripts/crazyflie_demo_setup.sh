#!/bin/bash

# Specify the workspace
WS='ws_multi_crazyflie_demo'

# Source the workspace
SW="source ~/$WS/devel/setup.bash"

tmux new-session -d -s cfdemo
tmux rename-window 'Crazyflie Demo'

tmux send-keys "$SW"
tmux send-keys Enter 
tmux send-keys 'roslaunch multi_mav_manager multi_crazyflie.launch sim:=0'
tmux send-keys Enter 

tmux split-window -h -t cfdemo
tmux send-keys "$SW"
tmux send-keys Enter 
tmux send-keys 'source ~/ws_multi_crazyflie_demo/src/multi_mav_manager/scripts/crazyflie_demo.sh'
tmux send-keys Enter 

tmux attach-session -t cfdemo
