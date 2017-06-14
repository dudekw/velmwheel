#include <urdf/model.h>
#include "ros/ros.h"
#include <iostream>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_parser");
  if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
std::cout<< argc<< std::endl;
std::cout<< argv[0]<< std::endl;
std::cout<< argv[1]<< std::endl;
  std::string urdf_file = argv[1];

  urdf::Model model;

  if (!model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
urdf::Joint my_joint();


std::cout<< model.getJoint("joint_base_laser_rear")->parent_to_joint_origin_transform.position.y << std::endl;
model.getJoint("joint_base_laser_rear")->parent_to_joint_origin_transform.position.y = 2;
  ROS_INFO("Successfully parsed urdf file");
  return 0;
}
