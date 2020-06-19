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
#include <manipulation_planning/plan_flower_sequence_action.hpp>

using namespace manipulation;

PlanFlowerSequenceAction::PlanFlowerSequenceAction(std::string _action_name, ros::NodeHandle &_nh, moveit_visual_tools::MoveItVisualTools _vt):
  action_server_(_nh, _action_name, boost::bind(&PlanFlowerSequenceAction::execute, this, _1), false),
  action_name_(_action_name),
  visual_tools_(_vt),
  nh_(_nh)
{
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
  //group_arm_->setPlanningTime(4.0); //Default was 10 or commented
  // group_arm_->setPlannerId("RRTConnectkConfigDefault");
  // group_arm_->setGoalOrientationTolerance(0.01); // 0.01 radians
  // group_arm_->setGoalPositionTolerance(0.01); // 0.01 m (radius of sphere where end effector must reach)
  group_arm_->setStartStateToCurrentState();
  group_arm_->setEndEffectorLink(ROBOT_TYPE + "_end_effector");

  // Subscribers
  // markers
  // sub_flower_poses_ = nh_.subscribe<geometry_msgs::PoseArray>("/bramblebee/arm/global_flower_poses", 1, &PlanFlowerSequenceAction::flower_poses_clk, this);
  // flowers
  sub_flower_poses_ = nh_.subscribe<manipulation_common::FlowerMap>("/flower_mapper/flower_map", 1, &PlanFlowerSequenceAction::flower_poses_clk, this);

  action_server_.registerGoalCallback(boost::bind(&PlanFlowerSequenceAction::goal_clk, this));

  action_server_.start();
}

void PlanFlowerSequenceAction::goal_clk()
{
  ROS_INFO("Goal callback");
}

void PlanFlowerSequenceAction::execute(const manipulation_planning::PlanFlowerSequenceGoal::ConstPtr &_goal)
{

  if (action_server_.isPreemptRequested() || !ros::ok())
  {
    ROS_WARN("Preemted Action: %s", action_name_.c_str());
    action_server_.setPreempted();
    return;
  }

  visual_tools_.deleteAllMarkers();
  visual_tools_.trigger();

  group_arm_->startStateMonitor();
  group_arm_->setStartStateToCurrentState();

  std::vector<int16_t> flower_ids(_goal->flower_ids);

  geometry_msgs::Pose first_search_pose;

  // retrieving Bramblebee's end-effector first search position
  double px, py, pz;
  nh_.getParam("/plan_flower_seq_action_node/first_search_pose/position/x", px);
  nh_.getParam("/plan_flower_seq_action_node/first_search_pose/position/y", py);
  nh_.getParam("/plan_flower_seq_action_node/first_search_pose/position/z", pz);

  double qx, qy, qz, qw;
  nh_.getParam("/plan_flower_seq_action_node/first_search_pose/orientation/x", qx);
  nh_.getParam("/plan_flower_seq_action_node/first_search_pose/orientation/y", qy);
  nh_.getParam("/plan_flower_seq_action_node/first_search_pose/orientation/z", qz);
  nh_.getParam("/plan_flower_seq_action_node/first_search_pose/orientation/w", qw);

  first_search_pose.position.x = px;
  first_search_pose.position.y = py;
  first_search_pose.position.z = pz;

  first_search_pose.orientation.x = qx;
  first_search_pose.orientation.y = qy;
  first_search_pose.orientation.z = qz;
  first_search_pose.orientation.w = qw;

  visual_tools_.publishAxis(first_search_pose);
  visual_tools_.trigger();

  std::vector<geometry_msgs::Pose> previsit_poses;
  previsit_poses.push_back(first_search_pose);
  std::cout << "getting desired poses" << std::endl;
  // markers
  // for (geometry_msgs::Pose flower_pose : flower_poses_)
  // {
  //   geometry_msgs::Pose pose = get_desired_previsit_ee_pose(flower_pose);
  //   previsit_poses.push_back(pose);
  // }

  // flowers
  // storing ids & poses in a map
  std::map<int, geometry_msgs::Pose> flower_map;
  flower_map[-1] = previsit_poses[0];

  std::cout << "flower_poses_.size(): " << flower_poses_.size() << std::endl;

  for (manipulation_common::Flower flower_pose : flower_poses_)
  {
    geometry_msgs::Pose pose = get_desired_previsit_ee_pose(flower_pose.pose);
    previsit_poses.push_back(pose);
    flower_map[flower_pose.id] = pose;
    visual_tools_.publishAxisLabeled(pose, "d" + std::to_string(flower_pose.id));
    visual_tools_.trigger();
  }

  // for (int i = 1; i < flower_ids.size() + 1; ++i)
  // {
  //   flower_map[flower_ids[i-1]] = previsit_poses[i];
  // }

  std::vector<std::vector<int16_t>> permutations;
  std::cout << "finding permutations" << std::endl;
  do {
    // for (int num : flower_ids)
    // {
    //   std::cout << " " << num;
    // }
    // std::cout << std::endl;
    permutations.push_back(flower_ids);
  } while (std::next_permutation(flower_ids.begin(), flower_ids.end()));

  // group_arm_->clearPathConstraints();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  std::vector<int> permutation_cost;
  int lowest_cost_index;
  int lowest_cost = 999999999;
  double start_time = ros::Time::now().toSec();
  for (int i = 0; i < permutations.size(); ++i)
  {
    // robot_state::RobotState start_state(*group_arm_->getCurrentState());
    robot_state::RobotState start_state(*(new robot_state::RobotState(robot_model_)));

    std::cout << "after getCurrentState" << std::endl;
    // robot_state::RobotStatePtr start_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup("arm");


    std::cout << "attempting: " << 0 << "->";
    std::vector<int16_t> order = permutations[i];

    // restart at search position
    bool set_IK = start_state.setFromIK(joint_model_group, flower_map[-1]);
    if (set_IK)
      group_arm_->setStartState(start_state);
    else
      ROS_ERROR("Couldn't set start state (outer loop)");

    int total_cost = 0;
    for (int j = 0; j < order.size(); ++j)
    {
      geometry_msgs::Pose pose = flower_map[order[j]];
      std::cout << order[j] << "->";
      group_arm_->setPoseTarget(pose);
      int cost;
      bool result = evaluate_plan(start_state, *group_arm_, cost);
      // std::cout << "cost " << cost << std::endl;
      total_cost = cost + total_cost;

      bool set_IK = start_state.setFromIK(joint_model_group, pose);
      if (set_IK)
        group_arm_->setStartState(start_state);
      else
        ROS_ERROR("Couldn't set start state (inner loop)");
    }

    permutation_cost.push_back(total_cost);

    if (lowest_cost > total_cost)
    {
      lowest_cost = total_cost;
      lowest_cost_index = i;
    }
    std::cout << "total_cost: " << total_cost << std::endl;

    if (ros::Time::now().toSec() - start_time > 60)
      break;
  }
  std::cout << "lowest_cost: " << lowest_cost << std::endl;
  std::cout << "lowest_cost_index: " << lowest_cost_index << std::endl;

  // sending result
  result_.sequence = permutations[lowest_cost_index];
  for (int i = 0; i < result_.sequence.size(); ++i)
  {
    geometry_msgs::Pose pose = flower_map[result_.sequence[i]];
    result_.ee_previsit_poses.push_back(pose);
  }

  action_server_.setSucceeded(result_);
}

bool PlanFlowerSequenceAction::evaluate_plan(const robot_state::RobotState &_start_state, moveit::planning_interface::MoveGroupInterface &_group, int &_cost)
{
  int count = 0;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  const robot_state::JointModelGroup *joint_model_group = _start_state.getJointModelGroup("arm");

  // reset flag for replan
  moveit::planning_interface::MoveItErrorCode result = moveit::planning_interface::MoveItErrorCode::FAILURE;

  // try to find a success plan.
  double plan_time;
  while (result != moveit::planning_interface::MoveItErrorCode::SUCCESS && count < 2)
  {
    ++count;
    // plan_time = count*10;
    // ROS_INFO("Setting plan time to %f sec", plan_time);
    // _group.setPlanningTime(plan_time);
    //_group.setPlannerId("RRTConnectkConfigDefault");
    result = _group.plan(my_plan);

    // rviz publish result trajectory
    visual_tools_.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools_.trigger();

    ros::WallDuration(0.1).sleep();
  }

  // plan not found
  if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    ROS_ERROR("Planning failed, reached max attempt: %d", count);
    _cost = 999;
    return false;
  }

  robot_trajectory::RobotTrajectoryPtr robot_trajectory(
        new robot_trajectory::RobotTrajectory(robot_model_, joint_model_group->getName()));

  robot_trajectory->setRobotTrajectoryMsg(_start_state, my_plan.trajectory_);

  // std::cout << "waypoint count: " << robot_trajectory->getWayPointCount() << std::endl;;

  _cost = robot_trajectory->getWayPointCount();
  ros::WallDuration(1.0).sleep();
  return true;
}

geometry_msgs::Pose PlanFlowerSequenceAction::get_desired_previsit_ee_pose(const geometry_msgs::Pose _flower_pose)
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

  std::cout << "des_pos:\n" << des_ee_pose << std::endl;
  return des_ee_pose;
}

// markers
// void PlanFlowerSequenceAction::flower_poses_clk(const geometry_msgs::PoseArray::ConstPtr &_poses)
// {
//   flower_poses_ = _poses->poses;
// }

// flowers
void PlanFlowerSequenceAction::flower_poses_clk(const manipulation_common::FlowerMap::ConstPtr &_poses)
{
  flower_poses_ = _poses->map;
}
