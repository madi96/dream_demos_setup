#!/bin/bash
xterm -e "sudo create_ap -n --no-virt wlan0 joystick_experiment joystickexp; $SHELL"& 
sleep 15
xterm -e " export ROS_IP=192.168.0.213 && roscore" & 
sleep 10 

xterm -e "export ROS_IP=192.168.0.213 && cd ~/modules_ws && . devel/setup.bash &&  rosrun dream_babbling_modules supervisor; $SHELL" & 
xterm -e "export ROS_IP=192.168.0.213 && cd ~/modules_ws && . devel/setup.bash && roslaunch mocap_optitrack mocap.launch; $SHELL" &
xterm -e "export ROS_IP=192.168.0.213 && cd ~/modules_ws && . devel/setup.bash && rosrun  dream_demos_setup optitrack_node; $SHELL"
