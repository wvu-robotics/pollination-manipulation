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
#include <manipulation_control/ee_go_to_pose_action.hpp>

using namespace manipulation;

EEGoToPoseAction::EEGoToPoseAction(std::string _action_name, ros::NodeHandle &_nh, moveit_visual_tools::MoveItVisualTools _vt):
  action_server_(_nh, _action_name, boost::bind(&EEGoToPoseAction::execute, this, _1), false),
  action_name_(_action_name),
  visual_tools_(_vt),
  nh_(_nh)
{
  nh_.param<std::string>("/robot_type", robot_type_, "j2n6s300");

  // Before we can load the planner, we need two objects, a RobotModel and a PlanningScene.
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = robot_model_loader.getModel();

  // Create Robot state and joint model group to keep track of current robot pose & planning
  robot_state_.reset(new robot_state::RobotState(robot_model_));

  // construct a `PlanningScene` that maintains the state of the world (including the robot).
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

  group_arm_ = new moveit::planning_interface::MoveGroupInterface("arm");
  group_arm_->startStateMonitor();
  group_arm_->setPlanningTime(20.0);   //default of 10.0
  group_arm_->setPlannerId("PRMstarkConfigDefault");
  group_arm_->setGoalOrientationTolerance(0.1); // 0.01 radians
  group_arm_->setGoalPositionTolerance(0.01); // 0.01 m (radius of sphere where end effector must reach)
  group_arm_->setStartStateToCurrentState();
  group_arm_->setEndEffectorLink(robot_type_ + "_end_effector");

  // Subscribers
  sub_start_pollination_ = nh_.subscribe<std_msgs::Bool>("/start_pollination_procedures", 1, &EEGoToPoseAction::start_pollination_clk, this);
  sub_joint_state_ = nh_.subscribe<sensor_msgs::JointState>("/"+ robot_type_ +"_driver/out/joint_state",  10, &EEGoToPoseAction::get_current_state_clk, this);

  // Publishers
  pub_collision_object_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
  pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  pub_cam_pose_ = nh_.advertise<manipulation_control::EECameraPose>("/bramblebee/arm/camera_pose", 10);

  // set up moveit's work scene
  clear_workscene();
  /**add_bramblebee_top_plate();
  // add_dev_table();
  add_brambleebee_arch();
  add_brambleebee_computer_box();**/

  //group_arm_->setNamedTarget("Retract");
  //group_arm_->move();
  //ros::WallDuration(0.5).sleep();

  action_server_.registerGoalCallback(boost::bind(&EEGoToPoseAction::goal_clk, this));

  action_server_.start();
}

void EEGoToPoseAction::goal_clk()
{
  ROS_INFO("Goal callback");
}

void EEGoToPoseAction::get_current_state_clk(const sensor_msgs::JointStateConstPtr &_msg)
{
  if (!action_server_.isActive())
    return;

  boost::mutex::scoped_lock lock(mutex_joint_state_); // prevents race conditions
  current_joint_state_ = *_msg;

  get_ee_pose();
}

void EEGoToPoseAction::get_ee_pose()
{
  const robot_state::JointModelGroup *joint_model_group = robot_model_->getJointModelGroup("arm");

  robot_state_->setJointGroupPositions(joint_model_group, current_joint_state_.position);
  ee_state_ = robot_state_->getGlobalLinkTransform(robot_type_ + "_end_effector");

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

void EEGoToPoseAction::start_pollination_clk(const std_msgs::Bool::ConstPtr &_msg)
{
  // if finished pollinating procedure
  if (start_pollination_ && !_msg->data)
  {
    visual_tools_.deleteAllMarkers();
    visual_tools_.trigger();

    //ROS_INFO("Moving back to retract position");
    //group_arm_->setNamedTarget("Retract");
    //group_arm_->move();
    ros::WallDuration(0.5).sleep();
  }

  start_pollination_ = _msg->data;
}

void EEGoToPoseAction::execute(const manipulation_control::EEGoToPoseGoal::ConstPtr &_goal)
{

  if (action_server_.isPreemptRequested() || !ros::ok())
  {
    ROS_WARN("Preemted Action: %s", action_name_.c_str());
    action_server_.setPreempted();
    return;
  }

  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  geometry_msgs::Pose goal_pose = _goal->goal_pose.pose;

  group_arm_->clearPathConstraints();
  group_arm_->setPoseTarget(goal_pose);

  /***
  * Visualizing plans in rviz
  ***/
  // ROS_INFO("Visualizing plan 1 as trajectory line");
  visual_tools_.publishAxisLabeled(goal_pose, "goal_pose");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools_.trigger();

  result_.goal_reached = evaluate_plan(*group_arm_);

  ROS_INFO("Action Completed: %s", action_name_.c_str());

  if (result_.goal_reached)
    action_server_.setSucceeded(result_);
  else
    action_server_.setAborted(result_);
}
bool EEGoToPoseAction::evaluate_plan(moveit::planning_interface::MoveGroupInterface &_group)
{
  int count = 0;
  //initial time setup, make sure it gets reset first time
  _group.setPlanningTime(5);
  _group.setPlannerId("PRMstarkConfigDefault");
  group_arm_->setGoalOrientationTolerance(0.1); // 0.01 radians
  group_arm_->setGoalPositionTolerance(0.01); 

  //seems like poseTarget gets wiped after bad planning, put this in to keep it in memory
  geometry_msgs::PoseStamped goal_pose_stamped = group_arm_->getPoseTarget();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  const robot_state::JointModelGroup *joint_model_group = robot_state_->getJointModelGroup("arm");
  bool result = (_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // reset flag for replan
  //moveit::planning_interface::MoveItErrorCode result = moveit::planning_interface::MoveItErrorCode::FAILURE;


  /***Testing to use only a single planner
*/
  // try to find a success plan.
  double plan_time, orientation_tolerance, position_tolerance;
  while (!result && count < 5)
  {
    //get_ee_pose();
      ++count;
      plan_time = count*10;
      orientation_tolerance = count*.01;
      position_tolerance = count*0.1;

      ROS_INFO("Setting plan time to %f sec, orientation tolerance to %f radians, and position tolerance to %f meters", plan_time,orientation_tolerance,position_tolerance);
      _group.setPlanningTime(plan_time);
      group_arm_->setGoalOrientationTolerance(0.1); // 0.01 radians
      group_arm_->setGoalPositionTolerance(0.01); 

      //make sure pose target is maintained properly
      group_arm_->setPoseTarget(goal_pose_stamped.pose);


      //_group.setPlannerId("RRTConnectkConfigDefault");
      result = (_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

      // rviz publish result trajectory
      //visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools_.trigger();

      ros::WallDuration(0.1).sleep();
  }
  

  // plan not found
  if (!result)
  {
    ROS_ERROR("Planning failed, reached max attempt: %d", count);
    return false;
  }

  // plan found
  _group.execute(my_plan);
  ros::WallDuration(1.0).sleep();
  return true;
}

void EEGoToPoseAction::clear_workscene()
{
    // remove table
    collision_object_msg_.id = "table";
    collision_object_msg_.operation = moveit_msgs::CollisionObject::REMOVE;

    collision_object_msg_.id = "plant";
    collision_object_msg_.operation = moveit_msgs::CollisionObject::REMOVE;

    collision_object_msg_.id = "bramblebee_top_plate";
    collision_object_msg_.operation = moveit_msgs::CollisionObject::REMOVE;

    collision_object_msg_.id = "bramblebee_arch";
    collision_object_msg_.operation = moveit_msgs::CollisionObject::REMOVE;

    collision_object_msg_.id = "bramblebee_computer_box";
    collision_object_msg_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_collision_object_.publish(collision_object_msg_);

    planning_scene_msg_.world.collision_objects.clear();
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);

    ros::WallDuration(1.0).sleep();
}
/**
void EEGoToPoseAction::add_dev_table()
{
  collision_object_msg_.header.frame_id = "root";
  collision_object_msg_.header.stamp = ros::Time::now();

  // add table
  collision_object_msg_.id = "table";
  collision_object_msg_.primitives.resize(1);
  collision_object_msg_.primitive_poses.resize(1);
  collision_object_msg_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_object_msg_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  collision_object_msg_.operation = moveit_msgs::CollisionObject::ADD;

  collision_object_msg_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.4;
  collision_object_msg_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.4;
  collision_object_msg_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;
  collision_object_msg_.primitive_poses[0].position.x = 0;
  collision_object_msg_.primitive_poses[0].position.y = 0.0;
  collision_object_msg_.primitive_poses[0].position.z = -0.03/2.0;
  pub_collision_object_.publish(collision_object_msg_);
  planning_scene_msg_.world.collision_objects.push_back(collision_object_msg_);
  planning_scene_msg_.is_diff = true;
  pub_planning_scene_diff_.publish(planning_scene_msg_);
  ros::WallDuration(0.1).sleep();
}

void EEGoToPoseAction::add_bramblebee_top_plate()
{
  shapes::Mesh *m = shapes::createMeshFromResource("package://manipulation_description/meshes/husky_top_plate.dae");

  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;
  shapes::constructMsgFromShape(m, co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
  collision_object_msg_.meshes.resize(1);
  collision_object_msg_.mesh_poses.resize(1);
  collision_object_msg_.meshes[0] = co_mesh;
  collision_object_msg_.id = "bramblebee_top_plate";

  collision_object_msg_.mesh_poses[0].position.x = 0;
  collision_object_msg_.mesh_poses[0].position.y = 0.28;
  collision_object_msg_.mesh_poses[0].position.z = -0.03/2.0;
  collision_object_msg_.mesh_poses[0].orientation.w = cos(-M_PI/4);
  collision_object_msg_.mesh_poses[0].orientation.x = 0.0;
  collision_object_msg_.mesh_poses[0].orientation.y = 0.0;
  collision_object_msg_.mesh_poses[0].orientation.z = sin(-M_PI/4);

  collision_object_msg_.meshes.push_back(co_mesh);
  collision_object_msg_.mesh_poses.push_back(collision_object_msg_.mesh_poses[0]);
  collision_object_msg_.operation = collision_object_msg_.ADD;

  pub_collision_object_.publish(collision_object_msg_);
  planning_scene_msg_.world.collision_objects.push_back(collision_object_msg_);

  planning_scene_msg_.is_diff = true;
  pub_planning_scene_diff_.publish(planning_scene_msg_);

  ros::WallDuration(0.1).sleep();
}

void EEGoToPoseAction::add_brambleebee_arch()
{
  shapes::Mesh *m = shapes::createMeshFromResource("package://manipulation_description/meshes/bramblebee_arch.dae");

  shape_msgs::Mesh co_mesh;
  shapes::ShapeMsg co_mesh_msg;
  shapes::constructMsgFromShape(m, co_mesh_msg);
  co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
  collision_object_msg_.meshes.resize(1);
  collision_object_msg_.mesh_poses.resize(1);
  collision_object_msg_.meshes[0] = co_mesh;
  collision_object_msg_.id = "bramblebee_arch";

  collision_object_msg_.mesh_poses[0].position.x = 0;
  collision_object_msg_.mesh_poses[0].position.y = 0.315 + 0.28;
  collision_object_msg_.mesh_poses[0].position.z = -0.03/2.0;
  collision_object_msg_.mesh_poses[0].orientation.w = cos(-M_PI/4);
  collision_object_msg_.mesh_poses[0].orientation.x = 0.0;
  collision_object_msg_.mesh_poses[0].orientation.y = 0.0;
  collision_object_msg_.mesh_poses[0].orientation.z = sin(-M_PI/4);

  collision_object_msg_.meshes.push_back(co_mesh);
  collision_object_msg_.mesh_poses.push_back(collision_object_msg_.mesh_poses[0]);

  collision_object_msg_.operation = collision_object_msg_.ADD;

  pub_collision_object_.publish(collision_object_msg_);
  planning_scene_msg_.world.collision_objects.push_back(collision_object_msg_);

  planning_scene_msg_.is_diff = true;
  pub_planning_scene_diff_.publish(planning_scene_msg_);

  ros::WallDuration(0.1).sleep();
}

void EEGoToPoseAction::add_brambleebee_computer_box()
{
  collision_object_msg_.primitives.resize(1);
  collision_object_msg_.primitive_poses.resize(1);
  collision_object_msg_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_object_msg_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  collision_object_msg_.operation = moveit_msgs::CollisionObject::ADD;

  collision_object_msg_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.51;
  //9.25 mm for lith-ion batteries
  collision_object_msg_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.325 + 0.0925;
  collision_object_msg_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.305;
  collision_object_msg_.primitive_poses[0].position.x = 0;
  collision_object_msg_.primitive_poses[0].position.y = 0.45 - (0.0925/2);
  collision_object_msg_.primitive_poses[0].position.z = 0.15;
  pub_collision_object_.publish(collision_object_msg_);

  planning_scene_msg_.world.collision_objects.push_back(collision_object_msg_);
  planning_scene_msg_.is_diff = true;
  pub_planning_scene_diff_.publish(planning_scene_msg_);
  ros::WallDuration(0.1).sleep();
}
**/