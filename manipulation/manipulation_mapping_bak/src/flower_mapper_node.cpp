#include <ros/ros.h>
#include <manipulation_mapping/flower_mapper.hpp>

int main(int argc, char **argv)
{
        ros::init(argc, argv, "flower_mapper_node");
        ROS_INFO("flower_mapper_node running...");
        manipulation::FlowerMapper flower_mapper;
        ros::spin();
        return 0;
}
