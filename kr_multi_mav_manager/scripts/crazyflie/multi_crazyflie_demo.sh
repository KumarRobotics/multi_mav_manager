#!/bin/bash

LOCATION=perch
CONFIG_PATH=$(rospack find kr_multi_mav_manager)/config/crazyflie/${LOCATION}
SCRIPTS_PATH=$(rospack find kr_multi_mav_manager)/scripts/crazyflie/

MAV_NAMESPACE=loco
MAV_MASS=0.04
ROTATE_WORLD=false

#Get all radio uris for MAVs from CSV
URI_FILENANME=${SCRIPTS_PATH}/radio_uris_${LOCATION}.csv
if [ ! -f "${URI_FILENANME}" ];
then
  echo "Please add radio uris in file ${URI_FILENANME}"
  exit 1
fi
declare -a MAV_RADIO_URIS
echo "Radio URIs"
while IFS=, read -r col1 col2
do
  echo "$col1|$col2"
  MAV_RADIO_URIS[$col1]=$col2
done < ${URI_FILENANME}

#Get MAV_IDS from CSV
#MAV_IDS=(4 5)
#readarray -d "," MAV_IDS < mav_ids.csv
if [ ! -f "${SCRIPTS_PATH}/mav_ids.csv" ];
then
  echo "Please add IDs of the MAVs to run in file mav_ids.csv"
  exit 1
fi
declare -a MAV_IDS
read line < ${SCRIPTS_PATH}/mav_ids.csv
echo "Line is : $line"
for i in $(echo $line | sed "s/,/ /g")
do
  MAV_IDS+=("$i")
done
NUM_MAV=${#MAV_IDS[@]}
echo 'Num MAVs' $NUM_MAV
echo 'Running mavs with ID' ${MAV_IDS[*]}

# Generate rviz config file for specific mav from default one
RVIZ_CONFIG_FILE="$HOME/.ros/wp_nav.rviz"
LAUNCH_PATH=$(rospack find kr_mav_launch)
cp $LAUNCH_PATH/launch/rviz_config.rviz ${RVIZ_CONFIG_FILE}
sed -i "s/quadrotor/temp/g" ${RVIZ_CONFIG_FILE}

# Generate multi_mav_manger yaml config file based on number of robots
cp $(rospack find kr_multi_mav_manager)/config/crazyflie/${LOCATION}/multi_mav_manager_empty.yaml ~/.ros/kr_multi_mav_manager.yaml
for id in ${MAV_IDS[*]}
do
  MAV_NAME=${MAV_NAMESPACE}${id}
  sed -i "1a\  '"${MAV_NAME}"'," ~/.ros/kr_multi_mav_manager.yaml
  echo "/${MAV_NAME}/active: true" >> ~/.ros/kr_multi_mav_manager.yaml
done

# Generate vicon launch file based on number of robots
cp $(rospack find mocap_vicon)/launch/vicon.launch ~/.ros/vicon.launch
sed -i "6a\    <param name=\"server_address\" value=\"mocap\"\/>" ~/.ros/vicon.launch
sed -i "6d" ~/.ros/vicon.launch
for id in ${MAV_IDS[*]}
do
  MAV_NAME=${MAV_NAMESPACE}${id}
  sed -i "12a\    <remap from=\"vicon/${MAV_NAME}/odom\" to=\"/${MAV_NAME}/odom\"\/>" ~/.ros/vicon.launch
done

RQT_GUI=rqt_multi_mav_gui
MASTER_URI=http://localhost:11311
SETUP_ROS_STRING="export ROS_MASTER_URI=${MASTER_URI}"
SESSION_NAME=demo_gs

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
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscore" Enter
tmux split-window -t $SESSION_NAME -h
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; export DISPLAY=:0; rqt --standalone ${RQT_GUI}" Enter
#tmux split-window -t $SESSION_NAME
#tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 3; export DISPLAY=:0; rosrun rviz rviz -d ${RVIZ_CONFIG_FILE}" Enter

tmux new-window -t $SESSION_NAME -n "Multi"
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; cd ~/.ros; roslaunch vicon.launch" Enter
tmux split-window -t $SESSION_NAME -h
tmux select-pane -t 0
tmux split-window -t $SESSION_NAME -v
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; roslaunch crazyflie_driver crazyflie_server.launch" Enter
tmux split-window -t $SESSION_NAME -v
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 5; roslaunch kr_multi_mav_manager multi_mav_manager.launch odom_topic:=odom config_path:=$HOME/.ros/" Enter
tmux select-pane -t 3
tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; roscd kr_multi_mav_manager/scripts/crazyflie; ./crazyflie_demo.sh"
# tmux select-layout -t $SESSION_NAME vertical

# Add window to easily kill all processes
tmux new-window -t $SESSION_NAME -n "Kill"
tmux send-keys -t $SESSION_NAME "tmux kill-session -t ${SESSION_NAME}"

for id in ${MAV_IDS[*]}
do
  MAV_NAME=${MAV_NAMESPACE}${id}
  echo $MAV_NAME ${MAV_RADIO_URIS[${id}]}
  tmux new-window -t $SESSION_NAME -n "r${id}"
  tmux send-keys -t $SESSION_NAME "$SETUP_ROS_STRING; sleep 6; roslaunch kr_multi_mav_manager crazyflie.launch model:=${MAV_NAME} uri:=${MAV_RADIO_URIS[${id}]} mass:=${MAV_MASS} config_path:=${CONFIG_PATH} rotate_world:=${ROTATE_WORLD}" Enter
done

tmux select-window -t $SESSION_NAME:1
tmux -2 attach-session -t $SESSION_NAME
