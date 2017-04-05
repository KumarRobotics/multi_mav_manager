#!/bin/bash

# Don't forget to source the setup.bash file.

SPACING=1.5

alias motors='rosservice call /multi_mav_services/motors'
alias takeoff='rosservice call /multi_mav_services/takeoff'
alias circle='rosservice call /multi_mav_services/goFormCircle "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"'
alias line='rosservice call /multi_mav_services/goFormLine "{roll: 0.0, pitch: 0.0, yaw: 0.0, center: [0.0, 0.0, 1.5], spacing: $SPACING}"'
alias land='rosservice call /multi_mav_services/land'
alias hover='rosservice call /multi_mav_services/hover'
alias eland='rosservice call /multi_mav_services/eland'
alias estop='rosservice call /multi_mav_services/estop'
alias stop='land && sleep 1 && motors false'
