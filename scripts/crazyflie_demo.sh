#!/bin/bash

# Don't forget to source the setup.bash file.
source ~/ws_multi_crazyflie_demo/devel/setup.bash

# This script assumes that you have launched:
#   roslaunch multi_mav_manager multi_crazyflie.launch sim:=0

# source ~/git/multi_mav_manager/scripts/set_crazyflie_gains.sh

SPACING=0.6

alias motors='rosservice call /multi_mav_services/motors'
alias takeoff='rosservice call /multi_mav_services/takeoff'
alias circle='rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"'
alias line='rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"'
alias angle='rosservice call /multi_mav_services/goFormAngle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"'
alias rect='rosservice call /multi_mav_services/goFormRect "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"'
alias land='rosservice call /multi_mav_services/land'
alias eland='rosservice call /multi_mav_services/eland'
alias estop='rosservice call /multi_mav_services/estop'


read -p "Press [Enter] to turn on motors"
rosservice call /multi_mav_services/motors true
sleep 1

read -p "Press [Enter] to takeoff"
rosservice call /multi_mav_services/takeoff
sleep 1

read -p "Press [Enter] to FORM CIRCLE at [0, 0, 0.7]m"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 0.7], spacing: $SPACING}"
sleep 1


read -p "Press [Enter] to EXPAND CIRCLE and MOVE to [0, 0, 2.5]m"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 2.5], spacing: 1.5}"
sleep 1

read -p "Press [Enter] to SHRINK CIRCLE and MOVE to [0, 0, 3]m"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 3.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to CHANGE to LINE without moving"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 3.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to CHANGE to RECTANGLE without moving"
rosservice call /multi_mav_services/goFormRect "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 3.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to CHANGE to CIRCLE and MOVE to [1 0 3]m"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [1.0, 0.0, 3.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to CHANGE to LINE, MOVE to [0, 0, 2], and PITCH by -0.25 rad"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: -0.25, yaw: 0.0, center: [0.0, 0.0, 2.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to CHANGE to LINE, MOVE to [0, 0, 1.5], and YAW by pi/2 rad (no pitch or roll)"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.00, yaw: 1.59, center: [0.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to MOVE to [1.5, 0, 1.5], and PITCH by -0.25 rad"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: -0.25, yaw: 1.59, center: [1.5, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to MOVE to [-1.5, 0, 1.5], and PITCH by 0.25 rad"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.25, yaw: 1.59, center: [-1.5, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to CHANGE to CIRCLE and MOVE to [0, 0, 1.5]m"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to EXPAND CIRCLE and MOVE to [0, 0, 1.0]m. This is the last shape before landing"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.0], spacing: 1.00}"
sleep 1

read -p "Press [Enter] to land"
rosservice call /multi_mav_services/land
sleep 1

read -p "Press [Enter] to turn off motors"
rosservice call /multi_mav_services/motors false
sleep 1
