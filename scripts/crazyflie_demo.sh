#!/bin/bash

# Don't forget to source the setup.bash file.
source ~/ws_multi_crazyflie_demo/devel/setup.bash

# This script assumes that you have launched:
#   roslaunch multi_mav_manager multi_crazyflie.launch sim:=0

# source ~/git/multi_mav_manager/scripts/set_crazyflie_gains.sh

SPACING=0.5

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

read -p "Press [Enter] to form circle. We can make them go into formation"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to expand circle. It can change"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: 1.5}"
sleep 1

read -p "Press [Enter] to shrink to a rectangle. And go back"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.5, yaw: 0.0, center: [1.5, 1.0, 2.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.5, pitch: 0.0, yaw: 0.0, center: [1.5, -1.0, 2.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: -0.5, yaw: 0.0, center: [0.5, -1.0, 2.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move circle"
rosservice call /multi_mav_services/goFormCircle "{roll: -0.5, pitch: 0.0, yaw: 0.0, center: [0.5, 1.0, 2.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to form rectangle"
rosservice call /multi_mav_services/goFormRect "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to make a pitched line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: -0.25, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to yaw the line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: -0.25, yaw: 1.59, center: [1.5, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move the line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.25, yaw: 1.59, center: [-1.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move the line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: -0.25, yaw: 1.59, center: [-1.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move the line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.0, yaw: 1.59, center: [-1.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to form circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [1.0, 0.0, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to form Rectangle"
rosservice call /multi_mav_services/goFormRect "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to land"
rosservice call /multi_mav_services/land
sleep 1

read -p "Press [Enter] to turn off motors"
rosservice call /multi_mav_services/motors false
sleep 1
