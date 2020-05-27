#!/bin/bash

#MAV_IDS=(4 5)
#Get MAV_IDS from CSV
#readarray -d "," MAV_IDS < mav_ids.csv
declare -a MAV_IDS
read line < mav_ids.csv
echo "Line is : $line"
for i in $(echo $line | sed "s/,/ /g")
do
    MAV_IDS+=("$i")
done
NUM_MAV=${#MAV_IDS[@]}

echo 'Num MAVs' $NUM_MAV
echo 'Running mavs with ID' ${MAV_IDS[*]}

# TODO parse this from command line? Possibly list of mav ids and namespace?
MAV_NAMESPACE=dragonfly

MASTER_URI=http://localhost:11311
SETUP_ROS_STRING="export ROS_MASTER_URI=${MASTER_URI}"
SESSION_NAME=tmux_swarm

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# Make mouse useful in copy mode
tmux setw -g mouse on

tmux rename-window -t $SESSION_NAME "Main"

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

for id in ${MAV_IDS[*]}
do

  MAV_NAME=${MAV_NAMESPACE}${id}

  tmux new-window -t $SESSION_NAME -n "r${id}"
  tmux send-keys -t $SESSION_NAME "ssh linaro@${MAV_NAME}" Enter
  tmux send-keys -t $SESSION_NAME "sudo -s" Enter
  tmux send-keys -t $SESSION_NAME "./tmux_tag.sh" Enter
  tmux select-layout -t $SESSION_NAME tiled
done

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME