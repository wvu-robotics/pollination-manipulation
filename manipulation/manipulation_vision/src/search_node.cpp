#include <ros/ros.h>
#include <manipulation_vision/search.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "search_node");
  ROS_INFO("search_node running...");
  manipulation::Search search;
  ros::spin();
  return 0;
}