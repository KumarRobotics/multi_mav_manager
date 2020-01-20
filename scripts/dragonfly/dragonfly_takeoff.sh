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

echo "Enable motors... ${ROBOT}"
rosservice call /multi_mav_services/motors true
read -p "Press [Enter] to takeoff "

echo "Takeoff... ${ROBOT}"
rosservice call /multi_mav_services/takeoff
read -p "Press [Enter] to go goTo "

echo "Sending relative z 0.45"
for id in ${MAV_IDS[*]}
do
  MAV_NAME=${MAV_NAMESPACE}${id}
  rosservice call /${MAV_NAME}/mav_services/goToRelative "goal:
  - 0.0
  - 0.0
  - 0.45
  - 0.0"
  sleep 0.1
done

sleep 0.2
echo "Call tag search"
for id in ${MAV_IDS[*]}
do
  MAV_NAME=${MAV_NAMESPACE}${id}
  rosservice call /${MAV_NAME}/tag_search "{}"
  sleep 0.1
done