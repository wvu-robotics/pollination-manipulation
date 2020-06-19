#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ros/ros.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/point_types.h>

// Note: for colorized point cloud, use PointXYZRGBA
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

// ros::Publisher pointcloud_pub;

//part of example code from https://github.com/tum-vision/rgbd_demo/blob/master/src/example.cpp
void callback(const ImageConstPtr& image_color_msg,
              const ImageConstPtr& image_depth_msg,
              const CameraInfoConstPtr& info_color_msg,
              const CameraInfoConstPtr& info_depth_msg) {

    // load images
    cv::Mat image_color = cv_bridge::toCvCopy(image_color_msg)->image;
    cv::Mat image_depth = cv_bridge::toCvCopy(image_depth_msg)->image;

    //intrinsics
    float fx = info_depth_msg->K[0];
    float fy = info_depth_msg->K[4];
    float cx = info_depth_msg->K[2];
    float cy = info_depth_msg->K[5];
    std::cout << "(fx, fy, cx, cy) = "<< fx << ", " << fy << ", " << cx << ", " << cy << std::endl;

    //compute xyz (3d coordinate in camera frame of reference)
    //note this is setup for if the depth image is U
    int u = image_depth_msg->width / 2;
    int v = image_depth_msg->height / 2;
    float depth = image_depth.at<short int>(cv::Point(u,v)) / 1000.0;
    float x = (x - cx) * depth / fx;
    float y = (y - cy) * depth / fy;
    float z = depth;
    std::cout << "(x, y, z) = " << x << ", " << y << ", " << z << std::endl;

    // produce a point cloud
    // PointCloud::Ptr pointcloud_msg (new PointCloud);
    // pointcloud_msg->header = image_depth_msg->header;

    // pcl::PointXYZ pt;
    // for(int y=0;y<image_color.rows;y+=4) {
    //     for(int x=0;x<image_color.cols;x+=4) {
    //         float depth = image_depth.at<short int>(cv::Point(x,y)) / 1000.0;

    //         if(depth>0) {
    //             pt.x = (x - cx) * depth / fx;
    //             pt.y = (y - cy) * depth / fy;
    //             pt.z = depth;
    //             //cout << pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
    //             pointcloud_msg->points.push_back (pt);
    //         }
    //     }
    // }
    // pointcloud_msg->height = 1;
    // pointcloud_msg->width = pointcloud_msg->points.size();
    // pointcloud_pub.publish (pointcloud_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;

    message_filters::Subscriber<Image> image_color_sub(nh,"/camera/color/image_raw", 1);
    message_filters::Subscriber<Image> image_depth_sub(nh,"/camera/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<CameraInfo> info_color_sub(nh,"/camera/color/camera_info", 1);
    message_filters::Subscriber<CameraInfo> info_depth_sub(nh,"/camera/aligned_depth_to_color/camera_info", 1);
    // pointcloud_pub = nh.advertise<PointCloud> ("mypoints", 1);

    typedef sync_policies::ApproximateTime<Image, Image, CameraInfo, CameraInfo> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_color_sub, image_depth_sub, info_color_sub, info_depth_sub);

    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    ros::spin();

    return 0;
}
