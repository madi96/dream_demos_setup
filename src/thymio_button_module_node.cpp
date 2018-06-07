/* 
Author       : Yaakoubi Oussama
Organism     : Amac-Isir (In the scoop of the Dream project)
Description  : This node is used to start/stop the thymio on buttonActivation
*/

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <dream_demos_setup/StampedBool.h>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>

// #define THYMIO_CONTROL_SCRIPT "\
// #/bin/bash \n\
// if [ $1 == 'run']
// then
//     sshpass -p 'Kenneth123!' scp run.txt admin@192.168.12.95:/home/lenilegoff/oussamaWorkspace/ 
// elif [ $1 == 'stop' ]
// then
//     sshpass -p 'Kenneth123!' scp stop.txt admin@192.168.12.95:/home/lenilegoff/oussamaWorkspace/
// else
//     echo 'Wrong param'
// fi
// "

bool buttonOn = false;
bool previousButtonState = false;

void buttonActivationCallback(const dream_demos_setup::StampedBool::ConstPtr& msg ){
  bool currentButtonState = msg->value;
  if (previousButtonState != currentButtonState){
      previousButtonState = currentButtonState;
      if ((currentButtonState) && !buttonOn){
	      ROS_INFO("Button Ativated: Sending the Run CMD To Thymio");
	      buttonOn = true;
        system("./scpSendRun.sh");
        ROS_INFO("Run CMD Executed");
  	    //system(THYMIO_CONTROL_SCRIPT 'run');
      } else if ((currentButtonState) && buttonOn){
	      ROS_INFO("Button Desativated: Sending the Stop CMD To Thymio");
        buttonOn = false;
        system("./scpSendStop.sh");
        ROS_INFO("Stop CMD Executed");
	      // system(THYMIO_CONTROL_SCRIPT 'stop');
      }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "button_module_node");  
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/ButtonModule_Blue", 100, buttonActivationCallback);
  ros::spin();

  return 0;
}
