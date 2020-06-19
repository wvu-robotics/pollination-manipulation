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
#include <manipulation_vision/approach_flower_action.hpp>

using namespace manipulation;

ApproachFlowerAction::ApproachFlowerAction(std::string _action_name, ros::NodeHandle &_nh, moveit_visual_tools::MoveItVisualTools _vt):
  action_server_(_nh, _action_name, boost::bind(&ApproachFlowerAction::execute, this, _1), false),
  action_name_(_action_name),
  visual_tools_(_vt),
  nh_(_nh)
{
  // get end effector pollinator (eep) offset (i.e. pollinator offset from end effector)
  double offset;
  nh_.getParam("/approach_flower_action_node/eep_x_offset", offset);
  eep_offset_.setX(offset);
  std::cout << "x: " <<  offset << std::endl;
  nh_.getParam("/approach_flower_action_node/eep_y_offset", offset);
  eep_offset_.setY(offset);
  std::cout << "y: " <<  offset << std::endl;
  nh_.getParam("/approach_flower_action_node/eep_z_offset", offset);
  eep_offset_.setZ(offset);
  std::cout << "z: " <<  offset << std::endl;

  // Before loading the planner, we need two objects, a RobotModel and a PlanningScene.
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = robot_model_loader.getModel();

  // Create Robot state and joint model group to keep track of current robot pose & planning
  robot_state_.reset(new robot_state::RobotState(robot_model_));

  // construct a `PlanningScene` that maintains the state of the world (including the robot).
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

  group_arm_ = new moveit::planning_interface::MoveGroupInterface("arm");
  group_arm_->startStateMonitor();
  group_arm_->setStartStateToCurrentState();
  group_arm_->setEndEffectorLink(ROBOT_TYPE + "_end_effector");

  // search service
  // searchForFlowersClient_ =
  // nh_.serviceClient<manipulation_common::SearchForFlowers>("/search");

  // Subscribers
  // TODO: Remove when CV for flowers working
  // sub_markers_ = nh_.subscribe<aruco_markers::MarkerArray>("/markers", 1, &ApproachFlowerAction::get_detected_markers_clk, this);
  sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/"+ ROBOT_TYPE +"_driver/out/joint_state",  10, &ApproachFlowerAction::get_current_state_clk, this);
  sub_flower_poses_ = nh_.subscribe<manipulation_common::FlowerMap>("/flower_mapper/flower_map", 1, &ApproachFlowerAction::flower_poses_clk, this);

  // Publishers
  pub_joint_vel_ = nh_.advertise<kinova_msgs::JointVelocity>("/j2n6s300_driver/in/joint_velocity", 1);
  pub_cam_pose_ = nh_.advertise<manipulation_control::EECameraPose>("/bramblebee/arm/camera_pose", 10);

  action_server_.registerGoalCallback(boost::bind(&ApproachFlowerAction::goal_clk, this));

  action_server_.start();
}

void ApproachFlowerAction::goal_clk()
{
  ROS_INFO("Goal callback");
}

void ApproachFlowerAction::execute(const manipulation_vision::ApproachFlowerGoal::ConstPtr &_goal)
{
  if (action_server_.isPreemptRequested() || !ros::ok())
  {
    ROS_WARN("Preemted Action: %s", action_name_.c_str());
    action_server_.setPreempted();
    return;
  }

  ROS_INFO("Executing approach flower action");
  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  // get end effector pose
  ros::spinOnce();
  ros::WallDuration(1.0).sleep();

  // // TODO: update when CV for flowers working
  // if (!current_detected_markers_.size())
  // {
  //   boost::shared_ptr<aruco_markers::MarkerArray const> msg_ptr = ros::topic::waitForMessage<aruco_markers::MarkerArray>("/markers", ros::Duration(3.0));
  //
  //   if ((msg_ptr->markers).size())
  //   {
  //     current_detected_markers_ = msg_ptr->markers;
  //     prev_detected_markers_ = current_detected_markers_;
  //   }
  //   else
  //   {
  //     ROS_WARN("No flower detected, can not do final approach");
  //     result_.flower_approached = false;
  //     action_server_.setSucceeded(result_, "No flower detected");
  //     return;
  //   }
  // }

  flower_id_ = _goal->flower_id;
  for (manipulation_common::Flower flower : current_detected_flowers_)
  {
    if (flower.id == flower_id_)
    {
      prev_detected_flower_ = flower;
      break;
    }
  }

  ROS_INFO("Begin approaching flower");
  double start_time = ros::Time::now().toSec();
  ros::Rate loop_rate(100);
  while (ros::ok() && !approach_flower(start_time))
  {
    // get updated end effector pose
    get_ee_pose();

    // //call search
    // if(searchForFlowersClient_.call(searchForFlowersSrv_))
    // {
    //   if(searchForFlowersSrv_.response.success == true)
    //   {
    //     ROS_INFO("SearchForFLowers call successful!");
    //   }
    //   else
    //   {
    //     ROS_ERROR("searchForFlowersSrv.success == false");
    //   }
    // }
    // else
    // {
    //   ROS_ERROR("Error! Failed to call service SearchForFlowers");
    // }

    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Finished executing approach flower action.");
  result_.flower_approached = true;
  action_server_.setSucceeded(result_);
  ros::WallDuration(1.0).sleep();
}


bool ApproachFlowerAction::approach_flower(const double _start_time)
{
  get_ee_pose();

  for (manipulation_common::Flower flower : current_detected_flowers_)
  {
    if (flower.id == flower_id_)
    {
      prev_detected_flower_ = flower;
      break;
    }
  }

  tf2::Vector3 rel_flower_position; // relative position
  rel_flower_position.setValue(prev_detected_flower_.point.point.x, prev_detected_flower_.point.point.y, prev_detected_flower_.point.point.z);

  if (prev_detected_flower_.point.point.z > EE_LOST_SIGHT_OFFSET)
  {
    ROS_INFO("Get previsit pose");
    geometry_msgs::Pose temp;
    temp.position = prev_detected_flower_.point.point;
    temp.orientation = tf2::toMsg(quat_g2ee_);
    // temp.orientation.x = quat_g2ee_.x;
    // temp.orientation.y = quat_g2ee_.y;
    // temp.orientation.z = quat_g2ee_.z;
    // temp.orientation.w = quat_g2ee_.w;
    des_previsit_pose_ = get_desired_previsit_ee_pose(temp);
  }

  // desired (end effector pose) quaternion rotation
  tf2::Quaternion quat_des_rot(des_previsit_pose_.orientation.x,
                               des_previsit_pose_.orientation.y,
                               des_previsit_pose_.orientation.z,
                              -des_previsit_pose_.orientation.w
                              );

  tf2::Matrix3x3 des_rot;
  des_rot.setRotation(quat_des_rot);

  tf2::Vector3 des_position(des_previsit_pose_.position.x,
                           des_previsit_pose_.position.y,
                           des_previsit_pose_.position.z
                          );

  // pollinator offset to flower
  tf2::Vector3 pos_offset(EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[0],
                          EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[1],
                          EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[2]
                         );

  tf2::Vector3 global_flower_position = des_position - pos_offset;

  // distance from end effector pointer (eep) in flower's reference frame
  tf2::Vector3 eep2flower_dist = des_rot * (global_flower_position - ee_xyz_) - eep_offset_;

  // parallel dist - move along xy plane until aligned with z=0
  tf2::Vector3 eep_dist_parallel(eep2flower_dist.x(), eep2flower_dist.y(), 0.0);

  // orth dist - distance in z direction (flower ref frame)
  tf2::Vector3 eep_dist_orthogonal(0.0, 0.0, eep2flower_dist.z());

  // distance from ee pointer to flower in global ref frame
  tf2::Vector3 eep_dist_global = des_rot.inverse() * eep2flower_dist;
  tf2::Vector3 eep_dist_parallel_global = des_rot.inverse() * eep_dist_parallel;
  tf2::Vector3 eep_dist_orthogonal_global = des_rot.inverse() * eep_dist_orthogonal;

  double parallel_dist = sqrt(eep_dist_parallel.x()*eep_dist_parallel.x()
                              + eep_dist_parallel.y()*eep_dist_parallel.y()
                             );

  // if (ros::Time::now().toSec() - _start_time > MAX_APPROACH_TIME)
  // {
  //   ROS_INFO("Reached max approach time. Flower is approached.");
  //   return true;
  // }

  Eigen::VectorXd ee_x_dot(6); // desired translational and angular velocity
  Eigen::VectorXd ee_vel(3); // desired translational velocity
  Eigen::VectorXd ee_angular_vel(3); // desired angular velocity
  Eigen::VectorXd joint_vel(6);

  // std::cout << "eep2flower_dist.z() = " <<  eep2flower_dist.z() << std::endl;
  // std::cout << "parallel_dist = " <<  parallel_dist << std::endl;

  // if parallel_dist < threshold then aligned with z axis
  if (parallel_dist < PARALLEL_DIST_THRESHOLD)
  {

    if (eep2flower_dist.z() < 0.015)
    {
      ROS_INFO("Touched flower.");
      return true;
    }

    ROS_INFO("Fixing distance");
    // later iteration - fixes distance
    ee_vel << eep_dist_global.x(), eep_dist_global.y(), eep_dist_global.z();
    ee_angular_vel << 0, 0, 0;
    ee_x_dot << ee_vel[0], ee_vel[1], ee_vel[2], ee_angular_vel[0], ee_angular_vel[1], ee_angular_vel[2];
  }
  else
  {
    ROS_INFO("Fixing parallel distance to desired");
    // std::cout << "parallel_dist = " <<  parallel_dist << std::endl;
    // std::cout << "eep2flower_dist.x = " <<  eep2flower_dist.x() << std::endl;
    // std::cout << "eep2flower_dist.y = " <<  eep2flower_dist.y() << std::endl;
    // std::cout << "eep2flower_dist.z = " <<  eep2flower_dist.z() << std::endl;

    // first iteration - ee_dist_parallel_global_ distance to desired
    ee_vel << eep_dist_parallel_global.x(), eep_dist_parallel_global.y(), eep_dist_parallel_global.z();
    ee_angular_vel << 0, 0, 0;
    ee_x_dot << ee_vel[0], ee_vel[1], ee_vel[2], ee_angular_vel[0], ee_angular_vel[1], ee_angular_vel[2];
  }

  // ROS_INFO("Inverting Jacobian");
  // ee_jacobian_.inverse() - matrix for transformation vel to joint vel

  if (abs((ee_jacobian_.inverse()).determinant()) > 100) // fixes local minima issue
  {
    ROS_INFO("Translation only servoing");

    // std::cout << "local min jacob inverse det:\n" << (ee_jacobian_.inverse()).determinant() << std::endl;
    joint_vel = ee_jacobian_.topRows(3).transpose()*(ee_jacobian_.topRows(3)*ee_jacobian_.topRows(3).transpose()).inverse() * ee_vel;
  }
  else
  {
    joint_vel = ee_jacobian_.inverse() * ee_x_dot;
  }

  // joint_vel - the rate of every joint that achieves ee_vel
  joint_vel.normalize(); // normalized so that all joints will move at a constant rate

  // ROS_INFO("Publishing joint velocity");
  kinova_msgs::JointVelocity joint_vel_msg;
  joint_vel_msg.joint1 = VELOCITY_SCALE * joint_vel[0];
  joint_vel_msg.joint2 = VELOCITY_SCALE * joint_vel[1];
  joint_vel_msg.joint3 = VELOCITY_SCALE * joint_vel[2];
  joint_vel_msg.joint4 = VELOCITY_SCALE * joint_vel[3];
  joint_vel_msg.joint5 = VELOCITY_SCALE * joint_vel[4];
  joint_vel_msg.joint6 = VELOCITY_SCALE * joint_vel[5];
  pub_joint_vel_.publish(joint_vel_msg); // publishes to a kinova topic (kinova moves it)
  return false;
}

geometry_msgs::Pose ApproachFlowerAction::get_desired_previsit_ee_pose(const geometry_msgs::Pose _flower_pose)
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

  visual_tools_.publishAxisLabeled(marker_pose, "f");
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

  visual_tools_.publishAxisLabeled(des_ee_pose, "d");
  visual_tools_.trigger();
  std::cout << "des_pos:\n" << des_ee_pose << std::endl;
  return des_ee_pose;
}

void ApproachFlowerAction::flower_poses_clk(const manipulation_common::FlowerMap::ConstPtr &_poses)
{
  current_detected_flowers_ = _poses->map;
}

void ApproachFlowerAction::get_current_state_clk(const sensor_msgs::JointStateConstPtr &_msg)
{
  if (!action_server_.isActive())
    return;

  boost::mutex::scoped_lock lock(mutex_joint_state_); // prevents race conditions
  current_joint_state_ = *_msg;

  get_ee_pose();
}

void ApproachFlowerAction::get_ee_pose()
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

  manipulation_control::EECameraPose cam_pose_msg;
  cam_pose_msg.quaternion = tf2::toMsg(quat_g2ee_);
  cam_pose_msg.position = tf2::toMsg(ee_xyz_);
  cam_pose_msg.header.stamp = ros::Time::now();
  cam_pose_msg.header.frame_id = "camera_pose";

  pub_cam_pose_.publish(cam_pose_msg);

  feedback_.current_camera_pose = cam_pose_msg;
  action_server_.publishFeedback(feedback_);
}

/* Aruco marker approach
bool ApproachFlowerAction::approach_flower(const double _start_time)
{
  get_ee_pose();
  // TODO: update when CV for flowers working

  if (!prev_detected_markers_.size())
    prev_detected_markers_ = current_detected_markers_;

  aruco_markers::Marker marker = prev_detected_markers_[0];
  for (aruco_markers::Marker m : prev_detected_markers_)
  {
    // the closest detected marker is the marker to apprach
    if (marker.tvec.z > m.tvec.z)
      marker = m;
  }


  tf2::Vector3 rel_flower_position; // relative position
  rel_flower_position.setValue(marker.tvec.x, marker.tvec.y, marker.tvec.z);

  if (marker.tvec.z > EE_LOST_SIGHT_OFFSET && current_detected_markers_.size())
  {
    ROS_INFO("Get previsit pose");
    des_previsit_pose_ = get_desired_previsit_ee_pose(marker);
  }

  // desired (end effector pose) quaternion rotation
  tf2::Quaternion quat_des_rot(des_previsit_pose_.orientation.x,
                               des_previsit_pose_.orientation.y,
                               des_previsit_pose_.orientation.z,
                              -des_previsit_pose_.orientation.w
                              );

  tf2::Matrix3x3 des_rot;
  des_rot.setRotation(quat_des_rot);

  tf2::Vector3 des_position(des_previsit_pose_.position.x,
                           des_previsit_pose_.position.y,
                           des_previsit_pose_.position.z
                          );

  // pollinator offset to flower
  tf2::Vector3 pos_offset(EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[0],
                          EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[1],
                          EE_OFFSET_FROM_FLOWER*des_rot.getRow(2)[2]
                         );

  tf2::Vector3 global_flower_position = des_position - pos_offset;

  // distance from end effector pointer (eep) in flower's reference frame
  tf2::Vector3 eep2flower_dist = des_rot * (global_flower_position - ee_xyz_) - eep_offset_;

  // parallel dist - move along xy plane until aligned with z=0
  tf2::Vector3 eep_dist_parallel(eep2flower_dist.x(), eep2flower_dist.y(), 0.0);

  // orth dist - distance in z direction (flower ref frame)
  tf2::Vector3 eep_dist_orthogonal(0.0, 0.0, eep2flower_dist.z());

  // distance from ee pointer to flower in global ref frame
  tf2::Vector3 eep_dist_global = des_rot.inverse() * eep2flower_dist;
  tf2::Vector3 eep_dist_parallel_global = des_rot.inverse() * eep_dist_parallel;
  tf2::Vector3 eep_dist_orthogonal_global = des_rot.inverse() * eep_dist_orthogonal;

  double parallel_dist = sqrt(eep_dist_parallel.x()*eep_dist_parallel.x()
                              + eep_dist_parallel.y()*eep_dist_parallel.y()
                             );

  // if (ros::Time::now().toSec() - _start_time > MAX_APPROACH_TIME)
  // {
  //   ROS_INFO("Reached max approach time. Flower is approached.");
  //   return true;
  // }

  Eigen::VectorXd ee_x_dot(6); // desired translational and angular velocity
  Eigen::VectorXd ee_vel(3); // desired translational velocity
  Eigen::VectorXd ee_angular_vel(3); // desired angular velocity
  Eigen::VectorXd joint_vel(6);

  // std::cout << "eep2flower_dist.z() = " <<  eep2flower_dist.z() << std::endl;
  // std::cout << "parallel_dist = " <<  parallel_dist << std::endl;

  // if parallel_dist < threshold then aligned with z axis
  if (parallel_dist < PARALLEL_DIST_THRESHOLD)
  {

    if (eep2flower_dist.z() < 0.01)
    {
      ROS_INFO("Touched flower.");
      return true;
    }

    ROS_INFO("Fixing distance");
    // later iteration - fixes distance
    ee_vel << eep_dist_global.x(), eep_dist_global.y(), eep_dist_global.z();
    ee_angular_vel << 0, 0, 0;
    ee_x_dot << ee_vel[0], ee_vel[1], ee_vel[2], ee_angular_vel[0], ee_angular_vel[1], ee_angular_vel[2];
  }
  else
  {
    ROS_INFO("Fixing parallel distance to desired");
    // std::cout << "parallel_dist = " <<  parallel_dist << std::endl;
    // std::cout << "eep2flower_dist.x = " <<  eep2flower_dist.x() << std::endl;
    // std::cout << "eep2flower_dist.y = " <<  eep2flower_dist.y() << std::endl;
    // std::cout << "eep2flower_dist.z = " <<  eep2flower_dist.z() << std::endl;

    // first iteration - ee_dist_parallel_global_ distance to desired
    ee_vel << eep_dist_parallel_global.x(), eep_dist_parallel_global.y(), eep_dist_parallel_global.z();
    ee_angular_vel << 0, 0, 0;
    ee_x_dot << ee_vel[0], ee_vel[1], ee_vel[2], ee_angular_vel[0], ee_angular_vel[1], ee_angular_vel[2];
  }

  // ROS_INFO("Inverting Jacobian");
  // ee_jacobian_.inverse() - matrix for transformation vel to joint vel

  if (abs((ee_jacobian_.inverse()).determinant()) > 100) // fixes local minima issue
  {
    ROS_INFO("Translation only servoing");

    // std::cout << "local min jacob inverse det:\n" << (ee_jacobian_.inverse()).determinant() << std::endl;
    joint_vel = ee_jacobian_.topRows(3).transpose()*(ee_jacobian_.topRows(3)*ee_jacobian_.topRows(3).transpose()).inverse() * ee_vel;
  }
  else
  {
    joint_vel = ee_jacobian_.inverse() * ee_x_dot;
  }

  // joint_vel - the rate of every joint that achieves ee_vel
  joint_vel.normalize(); // normalized so that all joints will move at a constant rate

  // ROS_INFO("Publishing joint velocity");
  kinova_msgs::JointVelocity joint_vel_msg;
  joint_vel_msg.joint1 = VELOCITY_SCALE * joint_vel[0];
  joint_vel_msg.joint2 = VELOCITY_SCALE * joint_vel[1];
  joint_vel_msg.joint3 = VELOCITY_SCALE * joint_vel[2];
  std::cout << "joint 3 vel: " << joint_vel[2] << std::endl;
  std::cout << "joint 3 scale*vel: " << VELOCITY_SCALE*joint_vel[2] << std::endl;
  joint_vel_msg.joint4 = VELOCITY_SCALE * joint_vel[3];
  joint_vel_msg.joint5 = VELOCITY_SCALE * joint_vel[4];
  joint_vel_msg.joint6 = VELOCITY_SCALE * joint_vel[5];
  pub_joint_vel_.publish(joint_vel_msg); // publishes to a kinova topic (kinova moves it)
  return false;
}
*/

// // Aruco marker approach
// geometry_msgs::Pose ApproachFlowerAction::get_desired_previsit_ee_pose(const aruco_markers::Marker _marker)
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
//   visual_tools_.deleteAllMarkers();
//   visual_tools_.publishAxis(marker_pose);
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
//   visual_tools_.publishAxis(des_ee_pose);
//   visual_tools_.trigger();
//
//   return des_ee_pose;
// }

// // ArUco marker approach
// void ApproachFlowerAction::get_detected_markers_clk(const aruco_markers::MarkerArray::ConstPtr &_msg)
// {
//   if (!action_server_.isActive())
//     return;
//
//   // ROS_INFO("Marker update");
//   current_detected_markers_ = _msg->markers;
//
//   if (!current_detected_markers_.size())
//     return;
//
//   aruco_markers::Marker marker = current_detected_markers_[0];
//   for (aruco_markers::Marker m : current_detected_markers_)
//   {
//     // the closest detected marker is the marker to apprach
//     if (marker.tvec.z > m.tvec.z)
//       marker = m;
//   }
//
//   // std::cout << "marker.tvec.z: " << marker.tvec.z << std::endl;
//
//   if (marker.tvec.z > EE_LOST_SIGHT_OFFSET)
//   {
//     prev_detected_markers_ = current_detected_markers_;
//     // ROS_INFO("Prev marker update");
//   }
// }
