//ros
#include <ros/ros.h>
#include <ros/package.h>

//image processing
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

//transforms
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>

//custom 
#include <manipulation_common/SearchForFlowers.h>
#include <manipulation_vision/SegmentFlowers.h>
#include <manipulation_vision/SegmentFlowersFF.h>
#include <manipulation_vision/Segment.h>
#include <manipulation_vision/ClassifyFlowers.h>
#include <manipulation_vision/ClassifyPose.h>
#include <manipulation_common/UpdateFlowerMap.h>

//pcl
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

namespace manipulation
{

class Search {
  public:
    Search();

    bool search (manipulation_common::SearchForFlowers::Request  &req,
                 manipulation_common::SearchForFlowers::Response &res);

    bool searchFF (manipulation_common::SearchForFlowers::Request  &req,
                   manipulation_common::SearchForFlowers::Response &res);

    ros::NodeHandle        nh;
    tf::TransformListener _listener;

    ros::ServiceServer searchServ;
    ros::ServiceServer searchFFServ;

    ros::ServiceClient segmentFlowersClient;
    ros::ServiceClient segmentFlowersFFClient;
    ros::ServiceClient classifyFlowersClient;
    ros::ServiceClient classifyPoseClient;
    ros::ServiceClient addFlowersToMapClient;

    manipulation_vision::SegmentFlowers  segmentFlowersSrv;
    manipulation_vision::SegmentFlowersFF  segmentFlowersFFSrv;
    manipulation_vision::ClassifyFlowers classifyFlowersSrv;
    manipulation_vision::ClassifyPose    classifyPoseSrv;

    manipulation_common::UpdateFlowerMap updateFlowerMapSrv;

  private:
    cv::Mat                 _rgb;
    cv::Mat                 _depth;
    sensor_msgs::CameraInfo _rgb_info;
    sensor_msgs::CameraInfo _depth_info;

    bool _load_rgb        (std::string topic);
    bool _load_depth      (std::string topic);

    bool _do_segmentation  ();
    bool _do_classification();

    sensor_msgs::PointCloud2 
    _compute_point_cloud (manipulation_vision::Segment segment);

    geometry_msgs::PoseStamped 
    _compute_pose(sensor_msgs::PointCloud2 ros_pc2);

    bool depth_constraint              (cv::Mat & rgb,
                                        cv::Mat & depth);

    bool _size_constraint_satisfied     (int       width,
                                         int       height);

    bool _distance_constraint_satisfied (geometry_msgs::PoseStamped pose);

    //method for debugging/analysis
    ros::Publisher pubSeg;

    bool republish();
    bool _publish_segments(std::vector<sensor_msgs::PointCloud2> points_clouds);
};

}
