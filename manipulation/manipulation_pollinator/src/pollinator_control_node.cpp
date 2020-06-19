#include <ros/ros.h>
#include <manipulation_pollinator/pollinator_control.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pollinator_control_node");
  ROS_INFO("pollinator_control_node running...");
  manipulation::PollinatorControl pollinator_control;
  pollinator_control.loadLookupTableMaestro();
  ros::spin();
  return 0;
}
