/* 
Author       : Yaakoubi Oussama
Organism     : Amac-Isir (In the scoop of the Dream project)
Description  : This node synchonises data from multiple buttons and then 
               updates the interface and controls the box Module  
*/


#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dream_demos_setup/StampedBool.h>

#include <sstream>
#include <string>
using namespace message_filters;

ros::Publisher interface_pub;
ros::Publisher box_pub ;


void interfaceButtonActivationCallback(const dream_demos_setup::StampedBool::ConstPtr& redButton, const dream_demos_setup::StampedBool::ConstPtr& greenButton){

  std_msgs::String msgToInterface;
  std_msgs::Bool msgToBox;
  if ((redButton->value) && (greenButton->value)){
    msgToInterface.data = "button:ON";
    msgToBox.data = true;
  }else{
    msgToInterface.data = "button:OFF";
    msgToBox.data = false;
  }
  box_pub.publish(msgToBox); 
  interface_pub.publish(msgToInterface);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "button_module_node");  
  ros::NodeHandle nh;
  interface_pub = nh.advertise<std_msgs::String>("/interface", 1000);
  box_pub = nh.advertise<std_msgs::Bool>("/BoxModule", 1000);


  message_filters::Subscriber<dream_demos_setup::StampedBool> squaredGreenButton_sub(nh, "ButtonModule_Red", 1);
  message_filters::Subscriber<dream_demos_setup::StampedBool> greenButton_sub(nh, "ButtonModule_Green", 1);


  typedef sync_policies::ApproximateTime<dream_demos_setup::StampedBool, dream_demos_setup::StampedBool> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), squaredGreenButton_sub,greenButton_sub);
  sync.registerCallback(boost::bind(&interfaceButtonActivationCallback, _1, _2));

  ros::spin();

  return 0;
}
