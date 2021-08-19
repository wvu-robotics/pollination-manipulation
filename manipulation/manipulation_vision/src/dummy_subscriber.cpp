#include <manipulation_vision/search.hpp>

void depthInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  
}

void colorInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{

}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{

}

void colorImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{

}

void pointCloudCallback(const pcl::PCLPointCloud2::ConstPtr& msg)
{

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_subscriber");
  ros::NodeHandle nh;
  ros::Subscriber depth_info_sub = nh.subscribe("/camera/depth/camera_info", 1, depthInfoCallback);
  ros::Subscriber color_info_sub = nh.subscribe("/camera/color/camera_info", 1, colorInfoCallback);
  ros::Subscriber depth_image_sub = nh.subscribe("/camera/depth/image_raw", 1, depthImageCallback);
  ros::Subscriber color_image_sub = nh.subscribe("/camera/color/image_raw", 1, colorImageCallback);
  ros::Subscriber point_cloud_sub = nh.subscribe("/camera/depth/color/points", 1, pointCloudCallback);

  ros::spin();

  return 0;
}
