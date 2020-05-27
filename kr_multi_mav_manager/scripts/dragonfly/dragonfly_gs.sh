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

RQT_GUI=rqt_multi_mav_gui

# TODO parse this from command line? Possibly list of mav ids and namespace?
MAV_NAMESPACE=dragonfly

MASTER_URI=http://localhost:11311
SETUP_ROS_STRING="export ROS_MASTER_URI=${MASTER_URI}"
SESSION_NAME=demo_gs${NUM_MAV}

if [ -z ${TMUX} ];
then
  TMUX= tmux new-session -s $SESSION_NAME -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# Generate rviz config file for specific mav from default one
RVIZ_CONFIG_FILE="$HOME/.ros/wp_nav.rviz"
LAUNCH_PATH=$(rospack find quadrotor_simulator)
cp $LAUNCH_PATH/launch/rviz_config.rviz ${RVIZ_CONFIG_FILE}
sed -i "s/quadrotor/temp/g" ${RVIZ_CONFIG_FILE}

# Generate multi_mav_manger yaml config file based on number of robots
cp $(rospack find kr_multi_mav_manager)/config/dragonfly/kr_multi_mav_manager_dragonfly.yaml ~/.ros/kr_multi_mav_manager.yaml
for id in ${MAV_IDS[*]}
do
  MAV_NAME=${MAV_NAMESPACE}${id}
  sed -i "1a\  '"${MAV_NAME}"'," ~/.ros/kr_multi_mav_manager.yaml
  echo "/${MAV_NAME}/active: true" >> ~/.ros/kr_multi_mav_manager.yaml
done

# Make mouse useful in copy mode
tmux setw -g mouse on

tmux rename-window -t $SESSION_NAME "Main"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscore" Enter
#tmux split-window -t $SESSION_NAME
#tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 3; export DISPLAY=:0; rosrun rviz rviz -d ${RVIZ_CONFIG_FILE}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 3; export DISPLAY=:0; rqt --standalone ${RQT_GUI}" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 3; roslaunch kr_multi_mav_manager dragonfly_manager.launch odom_topic:=odom_tag config_path:=$HOME/.ros/" Enter
tmux split-window -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 3; roslaunch kr_multi_mav_manager multi_dragonfly_gs.launch" Enter
tmux select-layout -t $SESSION_NAME tiled

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

tmux select-window -t $SESSION_NAME:0
tmux -2 attach-session -t $SESSION_NAME