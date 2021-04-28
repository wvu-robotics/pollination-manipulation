/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2019, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#ifndef BUILD_MAP_ACTION_HPP
#define BUILD_MAP_ACTION_HPP

#include <ros/ros.h>
#include <ros/console.h>

// actionlib
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

#include <manipulation_mapping/BuildMapAction.h>
#include <manipulation_control/EEGoToPoseAction.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h> // rviz_visual_tools
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h> // for obstacle avoidance

// shapes - used to create meshes in planning scene
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>

// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// pcl
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

// C++ standard libraries
#include <cmath> // trig functions
#define _USE_MATH_DEFINES // for math constants (M_PI)
#include <map>
#include <chrono> // high_resolution_clock, duration_cast

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <manipulation_control/EECameraPose.h>
#include <aruco_markers/MarkerArray.h>
#include <aruco_markers/Marker.h>

// Search
#include <manipulation_common/SearchForFlowers.h>
#include <manipulation_common/FlowerMap.h>
#include <manipulation_common/Flower.h>

namespace manipulation
{
/**
 * An Action Server that generates an obstacle map and detects flower/marker poses
 */
class BuildMapAction
{
public:
BuildMapAction(std::string action_name, ros::NodeHandle &nh, moveit_visual_tools::MoveItVisualTools vt);
~BuildMapAction(){
};

private:
ros::NodeHandle nh_;       /**< ROS node handle */
std::string robot_type_;        /**< Needed for Kinova API & MoveIt for motion planning */
std::string action_name_;       /**< Stored action name */
moveit_visual_tools::MoveItVisualTools visual_tools_;       /**< MoveIt visual tools used for visual aid. Used for visualizing projected pose estimates, planned move trajectories, etc */

// action dependencies
actionlib::SimpleActionServer<manipulation_mapping::BuildMapAction> action_server_;       /**< Action server for EE path planning to goal pose */
manipulation_mapping::BuildMapGoal goal_;       /**< Number of points to desample from pointcloud goal */
manipulation_mapping::BuildMapFeedback feedback_;       /**< End effector pose feedback*/
manipulation_mapping::BuildMapResult result_;       /**< Mapping complete result */
actionlib::SimpleActionClient<manipulation_control::EEGoToPoseAction> ee_go_to_pose_action_server_;       /**< Path planning action server, which accepts a flower pose goal */
int n_points_;       /**< number of sample points to downsample point cloud */

// search server
ros::ServiceClient searchForFlowersClient;
manipulation_common::SearchForFlowers searchForFlowersSrv;

// Subscribers
ros::Subscriber sub_joint_state_;       /**< Subscribes to the Kinova arm's JointState feedback from topic /j2n6s300_driver/out/joint_state */
ros::Subscriber sub_markers_;       /**< Subscribes to detected markers topic /markers */
ros::Subscriber sub_pointcloud_;       /**< Subscribes to topic /camera/depth/color/points that is the realsense pointcloud */
ros::Subscriber sub_flower_poses_;

// Publishers
ros::Publisher pub_cam_pose_;       /**< Publishes EE camera's quaternion and xyz to /bramblebee/arm/camera_pose & publishes as action server's feedback */
ros::Publisher pub_filtered_pointcloud_;       /**< Publishes to /bramblebee/arm/filtered_pointcloud, which is input for the octomap server for obstacle avoidance */
ros::Publisher pub_global_flower_poses_;       /**< Publishes global_flower_poses_ to /bramblebee/arm/global_flower_poses */
ros::Publisher pub_relative_flower_poses_;       /**< Publishes relative_flower_poses_ to /bramblebee/arm/relative_flower_poses */

boost::mutex mutex_joint_state_;       /**< used for synchronization */
sensor_msgs::JointState current_joint_state_;       /**< Current joint state is provided by Kinova that is used to find end effector pose */

// moveit
planning_scene::PlanningScenePtr planning_scene_;       /**< Planning scene that maintains state of the world including the robot */
planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;       /**< Subscribes to the planning_scene_ topic */
moveit::planning_interface::MoveGroupInterface *group_arm_;       /**< Bramblebee's arm group used for planning and control in MoveIt */
robot_model::RobotModelPtr robot_model_;       /**< Bramblebee's arm kinematic model */
robot_state::RobotStatePtr robot_state_;       /**< Bramblebee's current state (pose) */

// ee === end effector === camera
Eigen::Affine3d ee_state_;       /**< Transformation matrix of end effector */
Eigen::MatrixXd ee_jacobian_;       /**< Jacobian matrix of end effector */
tf2::Matrix3x3 rot_g2ee_;       /**< Rotation matrix from global to end effector's reference frame (retrieved from ee_state_) */
tf2::Quaternion quat_g2ee_;       /**< Quaternion from global to end effector's reference frame (retrieved from ee_rot_g2ee_) */
tf2::Vector3 ee_xyz_;       /**< XYZ position of end effector (retrieved from ee_state_) */

std::vector<geometry_msgs::PoseStamped> ee_search_poses_;       /**< Custom search position for Bramblebee's arm */

// Markers - TODO: update when CV for flowers working
std::vector<aruco_markers::Marker> current_detected_markers_;       /**< Detected markers that are currently seen */
std::map<int, geometry_msgs::PoseStamped> relative_flower_poses_;       /**< Map containing all detected markers, which contain the pose of the marker. The marker ID is the key to retrieve each marker. (relative to camera/end effector)*/
std::map<int, geometry_msgs::PoseStamped> global_flower_poses_;       /**< Map containing flowers that are needing to be visited. The marker ID is the key to retrieve the pose of the flower (in global coordinate frame) */

bool get_ee_pose_called_;       /**< Flag to check that the get_ee_pose() was called, for converting point cloud from relative to global */

int seq_count_;       /**< sequence id for publishing relative and global flower poses */

tf::TransformListener tf_listener_;       /**< Transform listener that subscribes to ROS transform messages. Used for transforming point cloud to global coordinates */

/**
 * Goal callback
 */
void goal_clk();

/**
 * Executive callback that executes a series of search end effector positions
 * to build an obstacle map & detect flowers/markers in the workspace
 */
void execute(const manipulation_mapping::BuildMapGoal::ConstPtr &goal);


/**
 * Converts detected flower/marker poses from relative to global coordinates & stores it in flowers_to_visit_
 * TODO: change param type when CV for flowers working
 */
geometry_msgs::PoseStamped convert_pose_to_global(const aruco_markers::Marker marker);

/**
 * Callback function for sub_markers_ subscriber to retrieve currently detected markers
 * TODO: update when CV for flowers working
 */
void get_detected_markers_clk(const aruco_markers::MarkerArray::ConstPtr &msg);

/**
 * Callback function for sub_pointcloud_ subscriber that ingests raw point cloud
 */
void pointcloud_clk(const sensor_msgs::PointCloud2::ConstPtr &msg);

/**
 * Callback function for sub_joint_state_ subscriber to retrieve the kinova arm's current joint states
 */
void get_current_state_clk(const sensor_msgs::JointStateConstPtr &msg);

/**
 * Retrieves end effector pose and publishes it to topic /bramblebee/arm/camera_pose
 */
void get_ee_pose();

/**
 * Retrieves various search poses for the end effector
 */
void get_search_poses();

void flower_poses_clk(const manipulation_common::FlowerMap::ConstPtr &poses);
std::vector<manipulation_common::Flower> flower_poses_;


};
}

#endif // BUILD_MAP_ACTION_HPP
