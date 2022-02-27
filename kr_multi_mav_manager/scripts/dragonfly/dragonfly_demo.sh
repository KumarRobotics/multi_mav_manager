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

MAV_NAMESPACE=dragonfly

NOMINAL_HEIGHT=1.5
SPACING=2.0

read -p "Press [Enter] to make a line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, $NOMINAL_HEIGHT], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move the line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [1.0, 0.0, $NOMINAL_HEIGHT], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move the line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: -0.0, yaw: 0.0, center: [-1.0, 0.0, $NOMINAL_HEIGHT], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to make a line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, $NOMINAL_HEIGHT], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to form circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, $NOMINAL_HEIGHT], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.1, yaw: 0.0, center: [1.0, 0.0, $NOMINAL_HEIGHT], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to form circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: -0.1, yaw: 0.0, center: [-1.5, 0.0, $NOMINAL_HEIGHT], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to move circle"
rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, $NOMINAL_HEIGHT], spacing: $SPACING}"
sleep 1

read -p "Press [Enter] to make a line"
rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.0], spacing: $SPACING}"
sleep 1