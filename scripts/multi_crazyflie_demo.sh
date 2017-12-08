#!/bin/bash

# Specify the workspace
WS='programs/ros/ws_multi_crazyflie_rotate_demo'

# Source the workspace
SW="source ~/$WS/devel/setup.bash"

tmux new-session -d -s cfdemo
tmux rename-window 'Crazyflie Demo'

tmux send-keys "$SW"
tmux send-keys Enter 
tmux send-keys 'roslaunch multi_mav_manager multi_crazyflie_rotate.launch sim:=0'
tmux send-keys Enter 

tmux split-window -h -t cfdemo
tmux send-keys "$SW"
tmux send-keys Enter 
tmux send-keys "land"

tmux split-window -v -t cfdemo
tmux send-keys "$SW"
tmux send-keys Enter
tmux send-keys 'source ~/programs/ros/ws_multi_crazyflie_rotate_demo/src/multi_mav_manager/scripts/crazyflie_demo.sh'
tmux send-keys Enter 

tmux attach-session -t cfdemo
