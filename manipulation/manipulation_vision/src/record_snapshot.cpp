//ros includes
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>

//image includes
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

//point cloud includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.h>

//other includes
#include <string>
#include <iostream>

const std::string point_cloud_topic = "/camera/depth/color/points";
const std::string image_topic = "/camera/color/image_raw";

//describes usage and parameters
void help()
{
  std::cout << "usage1: rosrun manipulation_vision record_snapshot" << std::endl;
  std::cout << "Defaults: " << std::endl;
  std::cout << "\t pointcloud topic = /camera/depth/color/points" << std::endl;
  std::cout << "\t image topic      = /camera/color/image_raw" << std::endl;
  // std::cout << "Parameters: " << std::endl;
  // std::cout << "\t ~prefix(str) -> Prefix for PCD file names created" << std::endl;
  // std::cout << "\t ~fixed_frame (str) -> If set, the transform from the fixed frame to the frame of the point cloud is written to the VIEWPOINT entry of the pcd file." << std::endl;
  // std::cout << "\t ~binary (bool, default: false) -> Output the pcd file in binary form." << std::endl;
  // std::cout << "\t ~compressed (bool, default: false) -> In case that binary output format is set, use binary compressed output." << std::endl << std::endl;
};

//This class is partially copied from the pcl_ros pointcloud_to_pcd.cpp
//However, this class was modified to record a single snapshot using
//waitForMessage rather than subscribing and using a callback function
class PointCloudToPCD
{
 protected:
   ros::NodeHandle nh_;

 private:
   std::string prefix_;
   bool binary_;
   bool compressed_;
   std::string fixed_frame_;
   tf2_ros::Buffer tf_buffer_;
   tf2_ros::TransformListener tf_listener_;

 public:
  void record_single()
  {
    pcl::PCLPointCloud2::ConstPtr cloud = ros::topic::waitForMessage<pcl::PCLPointCloud2> (point_cloud_topic, ros::Duration(1));

    if ((cloud->width * cloud->height) == 0)
    {
      return;
    }

    ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
              (int)cloud->width * cloud->height,
              cloud->header.frame_id.c_str (),
              pcl::getFieldsList (*cloud).c_str ());

    Eigen::Vector4f v = Eigen::Vector4f::Zero ();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity ();
    if (!fixed_frame_.empty ())
    {
      if (!tf_buffer_.canTransform (fixed_frame_, cloud->header.frame_id, pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration (3.0)))
      {
        ROS_WARN("Could not get transform!");
        return;
      }

      Eigen::Affine3d transform;
      transform = tf2::transformToEigen (tf_buffer_.lookupTransform (fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp)));
      v = Eigen::Vector4f::Zero ();
      v.head<3> () = transform.translation ().cast<float> ();
      q = transform.rotation ().cast<float> ();
    }

    std::stringstream ss;
    // ss << prefix_ << cloud->header.stamp << ".pcd";
    ss << "pc.pcd";
    ROS_INFO ("Data saved to %s", ss.str ().c_str ());

    pcl::PCDWriter writer;
    if(binary_)
    {
      if(compressed_)
      {
        writer.writeBinaryCompressed (ss.str (), *cloud, v, q);
      }
      else
      {
        writer.writeBinary (ss.str (), *cloud, v, q);
      }
    }
    else
    {
      writer.writeASCII (ss.str (), *cloud, v, q, 8);
    }
  } //end record_single

  PointCloudToPCD () : binary_(false), compressed_(false), tf_listener_(tf_buffer_)
  {
    // Check if a prefix parameter is defined for output file names.
    ros::NodeHandle priv_nh("~");
    if (priv_nh.getParam ("prefix", prefix_))
    {
      ROS_INFO_STREAM ("PCD file prefix is: " << prefix_);
    }
    else if (nh_.getParam ("prefix", prefix_))
    {
      ROS_WARN_STREAM ("Non-private PCD prefix parameter is DEPRECATED: " << prefix_);
    }

    priv_nh.getParam ("fixed_frame", fixed_frame_);
    priv_nh.getParam ("binary", binary_);
    priv_nh.getParam ("compressed", compressed_);
    if(binary_)
    {
      if(compressed_)
      {
        ROS_INFO_STREAM ("Saving as binary compressed PCD");
      }
      else
      {
        ROS_INFO_STREAM ("Saving as binary PCD");
      }
    }
    else
    {
      ROS_INFO_STREAM ("Saving as binary PCD");
    }

  } //end PointCloudToPCD
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_snapshot");
  help();

  //get filepath
  ROS_WARN("Snapshots will be recorded in working directory.");

  //record image
  std::cout << "Recording image..." << std::endl;
  sensor_msgs::Image::ConstPtr msg_rgb_ptr = ros::topic::waitForMessage<sensor_msgs::Image> (image_topic, ros::Duration(1));
  if(msg_rgb_ptr == NULL) //check if exists
  {
    ROS_ERROR("error: waitForMessage expired, rgb_image NULL");
    return -1;
  }

  cv_bridge::CvImagePtr cv_rgb_ptr = cv_bridge::toCvCopy(msg_rgb_ptr, "bgr8"); //convert to image pointer
  cv::imwrite("rgb.jpg", cv_rgb_ptr->image); //write to image to file

  //record pointcloud (xyzrgb)
  std::cout << "Recording pointcloud..." << std::endl;
  PointCloudToPCD obj;
  obj.record_single();

  return 0;
}
