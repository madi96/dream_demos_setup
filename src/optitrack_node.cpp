/* 
Author       : Yaakoubi Oussama based on the optitrack_handle implemented by S.K.
Organism     : Amac-Isir (In the scoop of the Dream project)
Description  : This node is used to start/stop the thymio on buttonActivation
*/


#include <ros/ros.h>
#include <ros/node_handle.h>

#include <std_msgs/Float64.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <dream_demos_setup/RoboboPose.h>

#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <stdio.h>
#include <fstream>
#include "quaternion.h"
#include <yaml-cpp/yaml.h>


Eigen::MatrixXd gRotToRobotBase(3,3);
Eigen::Vector3d gTransToRobotBase;

Eigen::Vector3d gCanPos, gRoboboPos, gJoystickPos;

ros::Publisher gRobobo_pos_quaternion_pub;
ros::Publisher gCan_pos_pub;
ros::Publisher gJoystick_pos_pub;
ros::Publisher gRobobo_pose_xyyaw_pub;

using namespace std;


int load_tf(const std::string& tf_file)
{
    // simple workaround
    std::ifstream tf_file_stream(tf_file.c_str());
    std::string line;

    char str [20];
    Eigen::VectorXd pos(3), quat(4);

    std::getline(tf_file_stream, line);
    std::sscanf(line.c_str(), "%s [%lf, %lf, %lf]",str, &pos(0), &pos(1), &pos(2) );

    std::getline(tf_file_stream, line);
    std::sscanf(line.c_str(), "%s [%lf, %lf, %lf, %lf]",str, &quat(1), &quat(2), &quat(3), &quat(0));

    tf_file_stream.close();

    for(int i=0; i<3; i++) gTransToRobotBase(i) = pos(i);

    Quaternion q(quat(0), quat(1), quat(2), quat(3));
    //std::cout << "Quaternion" << q << endl;
    q.normalize();

    gRotToRobotBase = q;
    std::cout << "gRotToRobotBase" << gRotToRobotBase << std::endl;
    std::cout << "tf pos : " << gTransToRobotBase << std::endl;
    std::cout << "tf rot : " << gRotToRobotBase << std::endl;
}

void transform_to_robotbase(const Eigen::Vector3d &pos_optitrack, Eigen::Vector3d &pos_robotbase)
{
    pos_robotbase = gRotToRobotBase*pos_optitrack + gTransToRobotBase;
    //pos_robotbase = gRotToRobotBase.transpose()*(pos_optitrack - gTransToRobotBase);
}


/*static void toEulerAngle(const Eigen::Quaternionf& q, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny, cosy);
}*/

void robobo_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& robobo_pose)
{
    Eigen::Vector3d robobo_position_optitrack;
    // make sure that optitrack streaming setting: up axis Z axis
    robobo_position_optitrack(0) = robobo_pose->pose.position.x;
    robobo_position_optitrack(1) = robobo_pose->pose.position.z;
    robobo_position_optitrack(2) = -robobo_pose->pose.position.y;
    transform_to_robotbase(robobo_position_optitrack, gRoboboPos);

    Eigen::Quaternionf robobo_orientation_optitrack;

    robobo_orientation_optitrack.x() = robobo_pose->pose.orientation.x;
    robobo_orientation_optitrack.z() = robobo_pose->pose.orientation.z;
    robobo_orientation_optitrack.y() = robobo_pose->pose.orientation.y;
    robobo_orientation_optitrack.w() = robobo_pose->pose.orientation.w;

    
    auto euler = robobo_orientation_optitrack.toRotationMatrix().eulerAngles(0, 1, 2);

    Eigen::Vector3d euler_vect;
    euler_vect(0) = euler(0);
    euler_vect(1) = euler(2);
    euler_vect(2) = -euler(1);
    Eigen::Vector3d euler_transf = gRotToRobotBase*euler_vect;

    // Added by Y.O. to get a yaw angle value in [-PI, -PI] instead of [-PI/2, PI/2]
    // depending on the value of pitch (or the roll)
    double yawZ = euler_transf(2);;
    if (euler_transf(2)>0){
      if (abs(euler_transf(1))>M_PI/2){
            yawZ = -M_PI - euler_transf(2);
        }
    }else if (euler_transf(2)<0){
  if (abs(euler_transf(1))>M_PI/2){
            yawZ = M_PI- euler_transf(2);
        }
    }


    //
    dream_demos_setup::RoboboPose roboboPose;
    roboboPose.x = gRoboboPos(0);
    roboboPose.y = gRoboboPos(1);
    roboboPose.yaw = yawZ;
    //cout << "Yaw angle :" <<yawZ*180/M_PI<< endl;
    gRobobo_pose_xyyaw_pub.publish(roboboPose);

    Eigen::Quaternionf q_baxter_frame;
    q_baxter_frame = Eigen::AngleAxisf(euler_transf(0), Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(euler_transf(1), Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(euler_transf(2), Eigen::Vector3f::UnitZ());

    geometry_msgs::PoseStamped position;
    position.header.stamp    = ros::Time::now();
    position.header.frame_id = "/base";
    position.pose.position.x = gRoboboPos(0);
    position.pose.position.y = gRoboboPos(1);
    position.pose.position.z = gRoboboPos(2);
    position.pose.orientation.x = q_baxter_frame.x();
    position.pose.orientation.y = q_baxter_frame.y();
    position.pose.orientation.z = q_baxter_frame.z();
    position.pose.orientation.w = q_baxter_frame.w();
    gRobobo_pos_quaternion_pub.publish(position);
}

void can_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& can_pose)
{
    Eigen::Vector3d can_position_optitrack;
    // make sure that optitrack streaming setting: up axis Z axis
    can_position_optitrack(0) = can_pose->pose.position.x;
    can_position_optitrack(1) = can_pose->pose.position.z;
    can_position_optitrack(2) = -can_pose->pose.position.y;
    transform_to_robotbase(can_position_optitrack, gCanPos);

    // Added by SD
    geometry_msgs::PoseStamped position;
    position.header.stamp    = ros::Time::now();
    position.header.frame_id = "/base";
    position.pose.position.x = gCanPos(0);
    position.pose.position.y = gCanPos(1);
    position.pose.position.z = gCanPos(2);
    gCan_pos_pub.publish(position);
}
// void toRobotFrame(const geometry_msgs::PoseStamped& optitrack_frame_pose, geometry_msgs::PoseStamped& robot_frame_pose){
//     tf::TransformListener listener;
//         ROS_INFO_STREAM("optitrack frame after transform x: "<<  optitrack_frame_pose.pose.position.x<<
//         " y: "<<  optitrack_frame_pose.pose.position.y<<
//         " z: "<<  optitrack_frame_pose.pose.position.z<< 
//         "frame_id: "<<optitrack_frame_pose.header.frame_id);
//     listener.transformPose("/base", optitrack_frame_pose, robot_frame_pose);
//     ROS_INFO_STREAM("robot frame after transform x: "<<  robot_frame_pose.pose.position.x<<
//         " y: "<<  robot_frame_pose.pose.position.y<<
//         " z: "<<  robot_frame_pose.pose.position.z<<
//         "frame_id: "<<robot_frame_pose.header.frame_id);

// }
void joystick_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& joystick_pose)
{

    // geometry_msgs::PoseStamped joystick_pose_optitrack_frame;
    // geometry_msgs::PoseStamped joystick_pose_robot_frame;   

    // joystick_pose_optitrack_frame.header.frame_id = joystick_pose->header.frame_id;
    // joystick_pose_optitrack_frame.header.stamp = joystick_pose->header.stamp;
    // joystick_pose_optitrack_frame.pose.position.x = joystick_pose->pose.position.x;
    // joystick_pose_optitrack_frame.pose.position.y = joystick_pose->pose.position.y;
    // joystick_pose_optitrack_frame.pose.position.z = joystick_pose->pose.position.z;

    // joystick_pose_optitrack_frame.pose.orientation.x = joystick_pose->pose.orientation.x;
    // joystick_pose_optitrack_frame.pose.orientation.y = joystick_pose->pose.orientation.y;
    // joystick_pose_optitrack_frame.pose.orientation.z = joystick_pose->pose.orientation.z;
    // joystick_pose_optitrack_frame.pose.orientation.w = joystick_pose->pose.orientation.w;

    // toRobotFrame(joystick_pose_optitrack_frame, joystick_pose_robot_frame);
    // gJoystick_pos_pub.publish(joystick_pose_robot_frame);

    Eigen::Vector3d joystick_position_optitrack;
    // make sure that optitrack streaming setting: up axis Z axis

    ROS_INFO_STREAM("befor transform x: "<<joystick_pose->pose.position.x<< 
        " y: "<<joystick_pose->pose.position.y<< " z: "<<joystick_pose->pose.position.z<<
         " frame_id: "<<joystick_pose->header.frame_id);
    joystick_position_optitrack(0) = joystick_pose->pose.position.x;
    joystick_position_optitrack(1) = joystick_pose->pose.position.y;
    joystick_position_optitrack(2) = joystick_pose->pose.position.z;
    transform_to_robotbase(joystick_position_optitrack, gJoystickPos);

    geometry_msgs::PoseStamped position;
    position.header.stamp    = ros::Time::now();
    position.header.frame_id = "/base";
    position.pose.position.x = gJoystickPos(0);
    position.pose.position.y = gJoystickPos(1);
    position.pose.position.z = gJoystickPos(2);
    ROS_INFO_STREAM("after transform x: "<<position.pose.position.x<<" y: "<<position.pose.position.y<< " z: "<<position.pose.position.z<<
         " frame_id: "<<position.header.frame_id);
    gJoystick_pos_pub.publish(position);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv,"contact_pose");
  ros::NodeHandle n;

  std::string tf_robotbase_file;
  n.getParam("tf_robotbase_file", tf_robotbase_file);

  load_tf(tf_robotbase_file);

  ros::Subscriber robobo_position_sub     = n.subscribe("/optitrack/robobo/pose", 1000, robobo_pose_callback);
  ros::Subscriber joystick_position_sub   = n.subscribe("/optitrack/joystick/pose", 1000, joystick_pose_callback);
  ros::Subscriber can_position_sub        = n.subscribe("/optitrack/can/pose", 1000, can_pose_callback);

  gJoystick_pos_pub                      = n.advertise<geometry_msgs::PoseStamped>("robot_frame_joystick_pose", 1, true);
  gRobobo_pos_quaternion_pub             = n.advertise<geometry_msgs::PoseStamped>("/robot_frame_robobo_pose_quaternion", 1, true);
  gRobobo_pose_xyyaw_pub                 = n.advertise<dream_demos_setup::RoboboPose>("/robot_frame_robobo_x_y_yaw", 1, true);
  gCan_pos_pub                           = n.advertise<geometry_msgs::PoseStamped>("/robot_frame_can_pose", 1, true);

  ros::spin();

  return 0;
}
