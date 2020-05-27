#!/bin/bash

# Don't forget to source the setup.bash file.
#source ~/programs/ros/ws_multi_crazyflie_rotate_demo/src/kr_multi_mav_manager/scripts/aliases.sh

# This script assumes that you have launched:
#   roslaunch kr_multi_mav_manager multi_crazyflie.launch sim:=0

# source ~/git/kr_multi_mav_manager/scripts/set_crazyflie_gains.sh

SPACING=0.7
CENTER_X=-5.5
CENTER_Y=0.0

alias motors='rosservice call /multi_mav_services/motors'
alias takeoff='rosservice call /multi_mav_services/takeoff'
alias circle='rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y,, 1.5], spacing: $SPACING}"'
alias circlel='rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y,, 1.0], spacing: $SPACING}"'
alias line='rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y,, 1.5], spacing: $SPACING}"'
alias angle='rosservice call /multi_mav_services/goFormAngle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y,, 1.5], spacing: $SPACING}"'
alias rect='rosservice call /multi_mav_services/goFormRect "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y,, 1.5], spacing: $SPACING}"'
alias land='rosservice call /multi_mav_services/land'
alias eland='rosservice call /multi_mav_services/eland'
alias estop='rosservice call /multi_mav_services/estop'

sleep 1

read -p "Press [Enter] to turn on motors and takeoff"
rosservice call /multi_mav_services/motors true
sleep 2

read -p "Press [Enter] to takeoff"
rosservice call /multi_mav_services/takeoff
sleep 1

read -p "Press [Enter] to form circle. We can make them go into formation"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move rectangle away"
rosservice call /multi_mav_services/goFormRect "{roll: 0.0, pitch: 0.4, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 2.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move Triangle towards"
rosservice call /multi_mav_services/goFormTriangle "{roll: 0.0, pitch: 0.7, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to expand circle. It can change"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 0.9], spacing: 0.8}"
sleep 1

read -p "Press [Enter] to Rotate"
rostopic pub -1 /rotate_on std_msgs/Bool "data: true"
sleep 1

read -p "Press [Enter] to expand circle. It can change"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 0.9], spacing: 1.35}"
sleep 1

read -p "Press [Enter] to expand circle. It can change"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 0.9], spacing: 0.8}"
sleep 1

read -p "Press [Enter] to form circle. We can make them go into formation"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.5, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to change to a rectangle"
rosservice call /multi_mav_services/goFormRect "{roll: 0.0, pitch: 0.6, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to change to a Triangle"
rosservice call /multi_mav_services/goFormTriangle "{roll: 0.0, pitch: 0.6, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 1.5], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to change to stairs"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.6, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 1.25], spacing: 0.5}"
sleep 1

read -p "Press [Enter] to make a christmas miracle"
rosservice call /multi_mav_services/goFormAngle "{roll: 0.0, pitch: 1.59, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 2.0], spacing: 0.6}"
sleep 1

read -p "Press [Enter] to form circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 1.0], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to Stop Rotate"
rostopic pub -1 /rotate_on std_msgs/Bool "data: false"
sleep 1

read -p "Press [Enter] to form low circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [$CENTER_X, $CENTER_Y, 0.1], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to land"
rosservice call /multi_mav_services/land
sleep 1

read -p "Press [Enter] to turn off motors"
rosservice call /multi_mav_services/motors false
sleep 1
