#include <ros/ros.h>
#include <ros/console.h>

#include <stdlib.h> // abs

// kinova
#include <kinova_msgs/JointVelocity.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h> // rviz_visual_tools

#include <geometric_shapes/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <actionlib/client/terminal_state.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h> // rviz_visual_tools
#include <string> // std::to_string()
#include <map>

// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PathPlanAction
#include <manipulation_control/EEGoToPoseAction.h> // action msg
#include <actionlib/client/simple_action_client.h>

#include <Eigen/Dense> // Matrix & Vectors
#include <Eigen/LU> // fullPivLu() //TODO: delete - prob not used
#include <Eigen/SVD> // svd

#include <manipulation_vision/ApproachFlowerAction.h>
#include <aruco_markers/MarkerArray.h>
#include <aruco_markers/Marker.h>

//moveit
// moveit_visual_tools::MoveItVisualTools visual_tools_("world"); /**< MoveIt visual tools used for visual aid. Used for visualizing projected pose estimates, planned move trajectories, etc */
robot_model::RobotModelPtr robot_model_; /**< Bramblebee's arm kinematic model */
robot_state::RobotStatePtr robot_state_; /**< Bramblebee's current state (pose) */
planning_scene::PlanningScenePtr planning_scene_; /**< Planning scene that maintains state of the world including the robot */
planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_; /**< Subscribes to the planning_scene_ topic */
moveit::planning_interface::MoveGroupInterface *group_arm_; /**< Bramblebee's arm group used for planning and control in MoveIt */
// boost::mutex mutex_joint_state_; /**< used for synchronization; prevents race conditions */
sensor_msgs::JointState current_joint_state_; /**< Current joint state is provided by Kinova that is used to find end effector pose */

Eigen::Affine3d ee_state_; /**< Transformation matrix of end effector */
Eigen::MatrixXd ee_jacobian_; /**< Jacobian matrix of end effector */
tf2::Matrix3x3 rot_g2ee_; /**< Rotation matrix from global to end effector's reference frame (retrieved from ee_state_) */
tf2::Quaternion quat_g2ee_; /**< Quaternion from global to end effector's reference frame (retrieved from ee_rot_g2ee_) */
tf2::Vector3 ee_xyz_; /**< XYZ position of end effector (retrieved from ee_state_) */
tf2::Vector3 eep_offset_; /**< end effector pollinator (eep) offset (i.e. pollinator offset from end effector) in meters */

const double EE_OFFSET_FROM_FLOWER = -0.2; /**< Desired end effector offset position from a flower */
const std::string ROBOT_TYPE = "j2n6s300";  /**< Needed for Kinova API & MoveIt to get ee pose */
const double EE_LOST_SIGHT_OFFSET = 0.215; /**< The offset in meters where the camera can no longer view the flower */
const double PARALLEL_DIST_THRESHOLD = 0.005; /**< Parallel distance threshold for when alignment with z axis */
const double MAX_APPROACH_TIME = 25.0; /**< Maximum time to allow approaching of flower */
const double VELOCITY_SCALE = 5.0; /**< Velocity scale for joints to move (e.g. 5deg/second)*/

// ros::Subscriber sub_joint_;
// ros::Subscriber sub_markers_;

geometry_msgs::Pose desired_pose_;

// void get_detected_markers_clk(const aruco_markers::MarkerArray::ConstPtr &_msg)
// {
//   current_detected_markers_ = _msg->markers;
//
//   if (!current_detected_markers_.size())
//     return;
//
//   desired_pose_ = get_desired_previsit_ee_pose(current_detected_markers_[0]);
// }

// Aruco markers
// geometry_msgs::Pose get_desired_previsit_ee_pose(const aruco_markers::Marker _marker)
// {
//   // ******** Calculates global marker pose ***************/
//   double angle = sqrt(_marker.rvec.x*_marker.rvec.x
//                       + _marker.rvec.y*_marker.rvec.y
//                       + _marker.rvec.z*_marker.rvec.z);
//   double x = _marker.rvec.x/angle;
//   double y = _marker.rvec.y/angle;
//   double z = _marker.rvec.z/angle;
//
//   // from camera to marker ref frame
//   tf2::Quaternion quat_marker_c2m(x*sin(angle/2),
//                                y*sin(angle/2),
//                                z*sin(angle/2),
//                                cos(angle/2));
//
//   tf2::Matrix3x3 rot_rel_marker;
//   rot_rel_marker.setRotation(quat_marker_c2m); // from cam to marker ref frame
//   rot_rel_marker = rot_rel_marker.transpose(); // from marker to cam
//   rot_rel_marker.getRotation(quat_marker_c2m);
//
//   // relative marker position in camera ref frame
//   tf2::Vector3 rel_marker_pos(_marker.tvec.x, _marker.tvec.y, _marker.tvec.z);
//   // from camera ref frame to global
//   tf2::Vector3 glob_marker_pos = ee_xyz_ + rot_g2ee_.transpose() * rel_marker_pos;
//
//   // from global to marker ref frame
//   tf2::Quaternion quat_marker_g2m = quat_marker_c2m * quat_g2ee_;
//   tf2::Matrix3x3 rot_marker_g2m;
//   rot_marker_g2m.setRotation(quat_marker_g2m);
//
//   // global coordinates marker pose
//   geometry_msgs::Pose marker_pose;
//   marker_pose.position.x = glob_marker_pos.x();
//   marker_pose.position.y = glob_marker_pos.y();
//   marker_pose.position.z = glob_marker_pos.z();
//   marker_pose.orientation.x = quat_marker_g2m.x();
//   marker_pose.orientation.y = quat_marker_g2m.y();
//   marker_pose.orientation.z = quat_marker_g2m.z();
//   marker_pose.orientation.w = -quat_marker_g2m.w();
//
//   // visual_tools_.publishAxis(marker_pose);
//   // visual_tools_.trigger();
//
//   // ******** Calculates desired global ee pose ***************/
//
//   tf2::Matrix3x3 des_rot(-rot_marker_g2m.getColumn(0)[0],
//                             -rot_marker_g2m.getColumn(1)[0],
//                             -rot_marker_g2m.getColumn(2)[0],
//                             rot_marker_g2m.getColumn(0)[1],
//                             rot_marker_g2m.getColumn(1)[1],
//                             rot_marker_g2m.getColumn(2)[1],
//                             -rot_marker_g2m.getColumn(0)[2],
//                             -rot_marker_g2m.getColumn(1)[2],
//                             -rot_marker_g2m.getColumn(2)[2]);
//
//   tf2::Quaternion quat_des_rot;
//   des_rot.getRotation(quat_des_rot);
//
//   tf2::Vector3 offset(EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[0],
//                       EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[1],
//                       EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[2]);
//
//   tf2::Vector3 des_pos = glob_marker_pos + offset;
//
//   geometry_msgs::Pose des_ee_pose;
//   des_ee_pose.position.x = des_pos.x();
//   des_ee_pose.position.y = des_pos.y();
//   des_ee_pose.position.z = des_pos.z();
//   des_ee_pose.orientation.x = quat_des_rot.x();
//   des_ee_pose.orientation.y = quat_des_rot.y();
//   des_ee_pose.orientation.z = quat_des_rot.z();
//   des_ee_pose.orientation.w = -quat_des_rot.w();
//
//   return des_ee_pose;
// }

geometry_msgs::Pose get_desired_previsit_ee_pose(const geometry_msgs::Pose _flower_pose)
{
  tf2::Quaternion quat_flower(_flower_pose.orientation.x, _flower_pose.orientation.y, _flower_pose.orientation.z,_flower_pose.orientation.w);

  // tf2::fromMsg(_flower_pose.orientation, quat_flower);
  tf2::Vector3 pos_flower(_flower_pose.position.x, _flower_pose.position.y, _flower_pose.position.z);
  // tf2::fromMsg(_flower_pose.position, pos_flower);

  tf2::Matrix3x3 rot_flower;
  rot_flower.setRotation(quat_flower);

  geometry_msgs::Pose marker_pose;
  marker_pose.position.x = pos_flower.x();
  marker_pose.position.y = pos_flower.y();
  marker_pose.position.z = pos_flower.z();
  marker_pose.orientation.x = quat_flower.x();
  marker_pose.orientation.y = quat_flower.y();
  marker_pose.orientation.z = quat_flower.z();
  marker_pose.orientation.w = quat_flower.w();

  // visual_tools_.publishAxisLabeled(marker_pose, "f");
  // visual_tools_.trigger();

  tf2::Matrix3x3 des_rot(rot_flower.getColumn(0)[0],
                            rot_flower.getColumn(1)[0],
                            rot_flower.getColumn(2)[0],
                            rot_flower.getColumn(0)[1],
                            rot_flower.getColumn(1)[1],
                            rot_flower.getColumn(2)[1],
                            rot_flower.getColumn(0)[2],
                            rot_flower.getColumn(1)[2],
                            rot_flower.getColumn(2)[2]);

  tf2::Matrix3x3 rot(-1, 0, 0,
                      0, 1, 0,
                      0, 0, -1);

  des_rot = rot*des_rot.transpose();

  tf2::Quaternion quat_des;
  des_rot.getRotation(quat_des);

  tf2::Vector3 offset(EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[0],
                      EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[1],
                      EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[2]);

  tf2::Vector3 des_pos = pos_flower + offset;

  geometry_msgs::Pose des_ee_pose;
  des_ee_pose.position.x = des_pos.x();
  des_ee_pose.position.y = des_pos.y();
  des_ee_pose.position.z = des_pos.z();
  des_ee_pose.orientation.x = quat_des.x();
  des_ee_pose.orientation.y = quat_des.y();
  des_ee_pose.orientation.z = quat_des.z();
  des_ee_pose.orientation.w = -quat_des.w();

  std::cout << "des_pos:\n" << des_ee_pose << std::endl;
  return des_ee_pose;
}

void get_ee_pose()
{
  const robot_state::JointModelGroup *joint_model_group = robot_model_->getJointModelGroup("arm");

  robot_state_->setJointGroupPositions(joint_model_group, current_joint_state_.position);
  ee_state_ = robot_state_->getGlobalLinkTransform(ROBOT_TYPE + "_end_effector");

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  robot_state_->getJacobian(joint_model_group,
                                robot_state_->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                reference_point_position, ee_jacobian_);

  rot_g2ee_.setValue(ee_state_.rotation()(0),
                        ee_state_.rotation()(1),
                        ee_state_.rotation()(2),
                        ee_state_.rotation()(3),
                        ee_state_.rotation()(4),
                        ee_state_.rotation()(5),
                        ee_state_.rotation()(6),
                        ee_state_.rotation()(7),
                        ee_state_.rotation()(8)
                        );

  rot_g2ee_.getRotation(quat_g2ee_);

  ee_xyz_.setValue(ee_state_.translation()[0],
                   ee_state_.translation()[1],
                   ee_state_.translation()[2]
                   );
}

void get_current_state_clk(const sensor_msgs::JointStateConstPtr &_msg)
{
  // boost::mutex::scoped_lock lock(mutex_joint_state_); // prevents race conditions
  current_joint_state_ = *_msg;

  get_ee_pose();
}

int main(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "test_action_node");
  ros::NodeHandle nh_;

  ros::Subscriber sub_joint_ = nh_.subscribe<sensor_msgs::JointState>("/"+ ROBOT_TYPE +"_driver/out/joint_state",  1, &get_current_state_clk);
  // sub_markers_ = nh_.subscribe<aruco_markers::MarkerArray>("/markers", 1, &get_detected_markers_clk);

  actionlib::SimpleActionClient<manipulation_control::EEGoToPoseAction> ee_go_to_pose_action_server_("ee_go_to_pose", true);
  geometry_msgs::PoseStamped search_pose;
  geometry_msgs::PoseStamped first_search_pose;
  first_search_pose.header.stamp = ros::Time::now();
  first_search_pose.pose.position.x = -0.177194014945;
  first_search_pose.pose.position.y = -0.688228885991;
  first_search_pose.pose.position.z = 0.393658605867;

  first_search_pose.pose.orientation.x = -0.707106651457;
  first_search_pose.pose.orientation.y = -2.33884735907e-07;
  first_search_pose.pose.orientation.z = -2.29391485123e-07;
  first_search_pose.pose.orientation.w = 0.707106948543;

  ROS_INFO("Waiting for action server...");
  ee_go_to_pose_action_server_.waitForServer();
  ROS_INFO("Action server started");

  ROS_INFO("Sending goal to path plan action server");
  manipulation_control::EEGoToPoseGoal goal;
  search_pose.pose = get_desired_previsit_ee_pose(first_search_pose.pose);
  goal.goal_pose = search_pose;
  ee_go_to_pose_action_server_.sendGoal(goal);

  ROS_INFO("Currently path planning..");

  bool finished_goal = false;

  finished_goal = ee_go_to_pose_action_server_.waitForResult();

  if (finished_goal && ee_go_to_pose_action_server_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Finished path planning to goal");
  }
  else
  {
    ROS_ERROR("Was not able to plan to goal");
  }

  manipulation_control::EEGoToPoseResult::ConstPtr result = ee_go_to_pose_action_server_.getResult();

  if (result->goal_reached)
  {
    ROS_INFO("Goal was reached!");
  }
  else
  {
    ROS_ERROR("Goal was not reached");
  }

  // // Before loading the planner, we need two objects, a RobotModel and a PlanningScene.
  // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // robot_model_ = robot_model_loader.getModel();
  //
  // // Create Robot state and joint model group to keep track of current robot pose & planning
  // robot_state_.reset(new robot_state::RobotState(robot_model_));
  //
  // // construct a `PlanningScene` that maintains the state of the world (including the robot).
  // planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  // planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  //
  // group_arm_ = new moveit::planning_interface::MoveGroupInterface("arm");
  // group_arm_->startStateMonitor();
  // group_arm_->setStartStateToCurrentState();
  // group_arm_->setEndEffectorLink("j2n6s300_end_effector");
  //
  //
  // ros::spinOnce();
  // ros::WallDuration(1.0).sleep();
  //
  // // boost::shared_ptr<aruco_markers::MarkerArray const> msg_ptr = ros::topic::waitForMessage<aruco_markers::MarkerArray>("/markers", ros::Duration(3.0));
  // //
  // // if (!(msg_ptr->markers).size())
  // // {
  // //   ROS_WARN_ONCE("No flowers detected.");
  // //   // msg_ptr = ros::topic::waitForMessage<aruco_markers::MarkerArray>("/markers", ros::Duration(3.0));
  // //   return -1;
  // // }
  //
  // desired_pose_ = get_desired_previsit_ee_pose(msg_ptr->markers[0]);
  //
  // geometry_msgs::PoseStamped pose;
  // pose.pose = desired_pose_;
  //
  // goal.goal_pose = pose;
  // ee_go_to_pose_action_server_.sendGoal(goal);
  //
  // ROS_INFO("Currently path planning to desired pose..");
  //
  // finished_goal = ee_go_to_pose_action_server_.waitForResult();
  //
  // if (finished_goal && ee_go_to_pose_action_server_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  // {
  //   ROS_INFO("Finished path planning to goal");
  // }
  // else
  // {
  //   ROS_ERROR("Was not able to plan to goal");
  // }
  //
  // manipulation_control::EEGoToPoseResult::ConstPtr result2 = ee_go_to_pose_action_server_.getResult();
  //
  // if (result2->goal_reached)
  // {
  //   ROS_INFO("Goal was reached!");
  // }
  // else
  // {
  //   ROS_ERROR("Goal was not reached");
  //   return 0;
  // }

  ros::WallDuration(1.0).sleep();

  actionlib::SimpleActionClient<manipulation_vision::ApproachFlowerAction> action_server_approach_flower_("approach_flower", true);

  ROS_INFO("Waiting for 2nd action server...");
  action_server_approach_flower_.waitForServer();
  ROS_INFO("2nd Action server started");

  ROS_INFO("Sending goal to approach flower server");
  manipulation_vision::ApproachFlowerGoal approach_goal;
  approach_goal.flower_id = 2;

  action_server_approach_flower_.sendGoal(approach_goal);

  ROS_INFO("Waiting for result");
  finished_goal = action_server_approach_flower_.waitForResult();

  ROS_INFO("Finished!");
  return 0;
}
