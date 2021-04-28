#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <manipulation_common/Flower.h>
#include <geometry_msgs/TransformStamped.h>
#include <manipulation_common/FlowerMap.h>
#include <manipulation_common/EigenDecomp.h>
#include <manipulation_common/UpdateFlowerMap.h>

#include <map>
#include <tuple>
#include <string>
#include <cassert>
#include <string>
#include <utility>
#include <iostream>
#include <functional>

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/BetweenFactor.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/lmeds.h>
#include <pcl/sample_consensus/mlesac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/rransac.h>
#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>



using namespace pcl;
using namespace pcl::io;
typedef SampleConsensusModelPlane<PointXYZ>::Ptr SampleConsensusModelPlanePtr;

namespace manipulation
{

class FlowerMapper {
public:
FlowerMapper();

bool update_flower_map (manipulation_common::UpdateFlowerMap::Request  &req,
                        manipulation_common::UpdateFlowerMap::Response &res);

bool clear_flower_map (manipulation_common::UpdateFlowerMap::Request  &req,
                       manipulation_common::UpdateFlowerMap::Response &res);



gtsam::Pose3 pose_msg_to_gtsam(const geometry_msgs::PoseStamped& fp);
unsigned int get_flower_index(const gtsam::Pose3& fp);
void update_graph(const unsigned int& flower_index, const gtsam::Point3 flower_position_est, const gtsam::Point3 flower_unit_vector, const double& flower_prob);
gtsam::Point3 get_pc_norm_vec(sensor_msgs::PointCloud2 pc);
void send_flower_map(manipulation_common::UpdateFlowerMap::Response &res, const ros::Publisher& pub);
void send_flower_vec(manipulation_common::UpdateFlowerMap::Response &res, const ros::Publisher& pub);
bool load_parameters(const ros::NodeHandle& nh);

void publish_point_clouds(std::vector<sensor_msgs::PointCloud2> points_clouds);

ros::NodeHandle nh;

ros::ServiceServer clearFlowerMapServ;
ros::ServiceServer constructFlowerMapServ;

// {flower_index, graph_key, factor graph, optimized values, number of times observed, accumulated prob. of flower, point cloud}
typedef std::vector< std::tuple<unsigned int, int, gtsam::NonlinearFactorGraph, gtsam::Values, unsigned int, double, sensor_msgs::PointCloud2> > factor_graphs;
// {flower_index,  flower_pose}
typedef std::vector< std::tuple<unsigned int, gtsam::Point3, gtsam::Point3> >flower_map;


private:

factor_graphs graphs_;
flower_map map_;
tf::TransformListener tf_listener_pc_;       /**< Transform listener that subscribes to ROS transform messages. Used for transforming point cloud to global coordinates */
tf::TransformListener tf_listener_pose_;       /**< Transform listener that subscribes to ROS transform messages. Used for transforming point cloud to global coordinates */
gtsam::Point3 normal_;


#define rad_2_deg_ (180.0/3.141592653589793238463)
sensor_msgs::PointCloud2 pc_;

ros::Publisher map_pub_;
ros::Publisher vec_pub_;
ros::Publisher pts_pub_;


typedef gtsam::noiseModel::Isotropic isoNoise;
typedef gtsam::noiseModel::Diagonal diagNoise;

// number of flowers currently represented in the map.
unsigned int flower_count_ = 0;

// filter noise params
double noise_position_classifier, noise_attitude_classifier, noise_position_process, noise_attitude_process, flower_distance_thresh;

bool position_only; // Only estimate position -- default is full 3d pose
bool make_robust; // place holder for now -- will add robust cost function later.
std::string world_frame;
double association_distance_thresh, model_distance_thresh;
int observation_thresh, estimator_type;

};

}
