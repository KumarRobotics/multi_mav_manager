#!/bin/bash

#MODELS='crazy05'
MODELS='crazy01 crazy02 crazy03 crazy04 crazy05 crazy06'

# Roll and pitch gains
KP=10
KI=4
KD=0

# Roll and pitch rate gains
RATE_KP=250
RATE_KI=500
RATE_KD=2.5

#################
## Pitch gains ##
#################

for MODEL in $MODELS; do

  echo 'Setting gains for ' $MODEL

  ################
  ## Roll gains ##
  ################

  printf '\nCurrent Roll kp, ki, and kd\n'
  rosparam get /$MODEL/pid_attitude/roll_kp
  rosparam get /$MODEL/pid_attitude/roll_ki
  rosparam get /$MODEL/pid_attitude/roll_kd

  rosparam set /$MODEL/pid_attitude/roll_kp $KP
  rosparam set /$MODEL/pid_attitude/roll_ki $KI 
  rosparam set /$MODEL/pid_attitude/roll_kd $KD 

  printf '\nNew Roll kp, ki, and kd\n'
  rosparam get /$MODEL/pid_attitude/roll_kp
  rosparam get /$MODEL/pid_attitude/roll_ki
  rosparam get /$MODEL/pid_attitude/roll_kd  

  #################
  ## Pitch gains ##
  #################

  printf '\nCurrent Pitch kp, ki, and kd\n'
  rosparam get /$MODEL/pid_attitude/pitch_kp
  rosparam get /$MODEL/pid_attitude/pitch_ki
  rosparam get /$MODEL/pid_attitude/pitch_kd

  rosparam set /$MODEL/pid_attitude/pitch_kp $KP
  rosparam set /$MODEL/pid_attitude/pitch_ki $KI 
  rosparam set /$MODEL/pid_attitude/pitch_kd $KD 

  printf '\nNew Pitch kp, ki, and kd\n'
  rosparam get /$MODEL/pid_attitude/pitch_kp
  rosparam get /$MODEL/pid_attitude/pitch_ki
  rosparam get /$MODEL/pid_attitude/pitch_kd  

  ######################
  ## Pitch rate gains ##
  ######################
 
  printf '\nCurrent Pitch Rate kp, ki, and kd\n'
  rosparam get /$MODEL/pid_rate/pitch_kp
  rosparam get /$MODEL/pid_rate/pitch_ki
  rosparam get /$MODEL/pid_rate/pitch_kd
  
  rosparam set /$MODEL/pid_rate/pitch_kp $RATE_KP
  rosparam set /$MODEL/pid_rate/pitch_ki $RATE_KI 
  rosparam set /$MODEL/pid_rate/pitch_kd $RATE_KD 
    
  printf '\nNew Pitch Rate kp, ki, and kd\n'
  rosparam get /$MODEL/pid_rate/pitch_kp
  rosparam get /$MODEL/pid_rate/pitch_ki
  rosparam get /$MODEL/pid_rate/pitch_kd
  
  #####################
  ## Roll rate gains ##
  #####################
  
  printf '\nCurrent Roll Rate kp, ki, and kd\n'
  rosparam get /$MODEL/pid_rate/roll_kp
  rosparam get /$MODEL/pid_rate/roll_ki
  rosparam get /$MODEL/pid_rate/roll_kd
  
  rosparam set /$MODEL/pid_rate/roll_kp $RATE_KP
  rosparam set /$MODEL/pid_rate/roll_ki $RATE_KI
  rosparam set /$MODEL/pid_rate/roll_kd $RATE_KD
 
  printf '\nNew Roll Rate kp, ki, and kd\n'
  rosparam get /$MODEL/pid_rate/roll_kp
  rosparam get /$MODEL/pid_rate/roll_ki
  rosparam get /$MODEL/pid_rate/roll_kd

  ####################
  ## Yaw Rate gains ##
  ####################
  
  printf '\nCurrent Yaw Rate kp, ki, and kd\n'
  rosparam get /$MODEL/pid_rate/yaw_kp
  rosparam get /$MODEL/pid_rate/yaw_ki
  rosparam get /$MODEL/pid_rate/yaw_kd
  
  rosparam set /$MODEL/pid_rate/yaw_kp 160
  rosparam set /$MODEL/pid_rate/yaw_ki 0
  rosparam set /$MODEL/pid_rate/yaw_kd 0
    
  printf '\nNew Yaw Rate kp, ki, and kd\n'
  rosparam get /$MODEL/pid_rate/yaw_kp
  rosparam get /$MODEL/pid_rate/yaw_ki
  rosparam get /$MODEL/pid_rate/yaw_kd

  ## Update all params
  rosservice call /$MODEL/update_params "params: [
    'pid_attitude/pitch_kp',
    'pid_attitude/pitch_ki',
    'pid_attitude/pitch_kd',
    'pid_attitude/roll_kp',
    'pid_attitude/roll_ki',
    'pid_attitude/roll_kd',
    'pid_rate/pitch_kp',
    'pid_rate/pitch_ki',
    'pid_rate/pitch_kd',
    'pid_rate/roll_kp',
    'pid_rate/roll_ki',
    'pid_rate/roll_kd',
    'pid_rate/yaw_kp',
    'pid_rate/yaw_ki',
    'pid_rate/yaw_kd']"

done
