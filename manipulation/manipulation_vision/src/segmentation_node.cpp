#include <ros/ros.h>
#include <manipulation_vision/segmentation.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation_node");
  ROS_INFO("segmentation_node running...");
  manipulation::Segmentation segmentation;
  segmentation.loadLookupTableRGB();
  ros::spin();
  return 0;
}