#!/bin/bash

# Don't forget to source the setup.bash file.
source /home/demo/programs/ros/multi_ws/devel/setup.bash

# This script assumes that you have launched:
#   roslaunch kr_multi_mav_manager multi_hummingbird.launch sim:=0 launch_mocap:=1

SPACING=1.5

alias motors='rosservice call /multi_mav_services/motors'
alias takeoff='rosservice call /multi_mav_services/takeoff'
alias circle='rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"'
alias line='rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"'
alias land='rosservice call /multi_mav_services/land'
alias eland='rosservice call /multi_mav_services/eland'
alias estop='rosservice call /multi_mav_services/estop'

read -p "Press [Enter] to turn on motors"
rosservice call /multi_mav_services/motors true
sleep 1

read -p "Press [Enter] to takeoff"
rosservice call /multi_mav_services/takeoff
sleep 1

read -p "Press [Enter] to form circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.4, yaw: 0.0, center: [4.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to make a line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: -0.2, yaw: 0.0, center: [-2.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move the line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to form circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [3.0, 0.0, 2.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.4, yaw: 0.0, center: [0.0, 0.0, 1.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to make a line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: -0.2, yaw: 0.0, center: [2.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to land"
rosservice call /multi_mav_services/land
sleep 1

read -p "Press [Enter] to turn off motors"
rosservice call /multi_mav_services/motors false
sleep 1
