#include <ros/ros.h>
#include <manipulation_vision/classification.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "classification_node");
  ROS_INFO("classification_node running...");
  manipulation::Classification classification;
  ros::spin();
  return 0;
}