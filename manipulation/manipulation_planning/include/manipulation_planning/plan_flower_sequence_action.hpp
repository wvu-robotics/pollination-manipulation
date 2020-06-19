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
#ifndef PLAN_FLOWER_SEQUENCE_ACTION_HPP
#define PLAN_FLOWER_SEQUENCE_ACTION_HPP

#include <ros/ros.h>

// actionlib
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h> // rviz_visual_tools
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS msgs
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

// custom msgs
#include <manipulation_planning/PlanFlowerSequenceAction.h>
#include <manipulation_common/FlowerMap.h>
#include <manipulation_common/Flower.h>

#include <Eigen/Core> // Matrix & Vectors
#include <vector>

namespace manipulation
{
  /**
   *  An action server that plans the sequence of flowers to pollinate
   */
  class PlanFlowerSequenceAction
  {
    public:
      PlanFlowerSequenceAction(std::string action_name, ros::NodeHandle &nh, moveit_visual_tools::MoveItVisualTools vt);
      ~PlanFlowerSequenceAction(){};

    private:
      const std::string ROBOT_TYPE = "j2n6s300";  /**< Needed for Kinova API & MoveIt to get ee pose */
      const double EE_OFFSET_FROM_FLOWER = -0.20; /**< Desired end effector offset position from a flower */


      ros::NodeHandle nh_; /**< ROS node handle */
      std::string action_name_; /**< Stored action name */

      // Subscribers
      ros::Subscriber sub_flower_poses_;

      //markers
      // std::vector<geometry_msgs::Pose> flower_poses_;
      // flowers
      std::vector<manipulation_common::Flower> flower_poses_;

      // action dependencies
      actionlib::SimpleActionServer<manipulation_planning::PlanFlowerSequenceAction> action_server_; /**< Action server for EE path planning to goal pose */
      manipulation_planning::PlanFlowerSequenceGoal goal_; /**< Flower sequence goal */
      manipulation_planning::PlanFlowerSequenceFeedback feedback_; /**< Flower sequence feedback*/
      manipulation_planning::PlanFlowerSequenceResult result_; /**< Flower sequence result */

      moveit_visual_tools::MoveItVisualTools visual_tools_; /**< MoveIt visual tools used for visual aid. Used for visualizing projected pose estimates, planned move trajectories, etc */
      moveit::planning_interface::MoveGroupInterface *group_arm_; /**< Bramblebee's arm group used for planning and control in MoveIt */
      robot_model::RobotModelPtr robot_model_; /**< Bramblebee's arm kinematic model */
      robot_state::RobotStatePtr robot_state_; /**< Bramblebee's current state (pose) */
      moveit::planning_interface::MoveItErrorCode moveit_result_; /**< Result from attempt to move to a pose */
      planning_scene::PlanningScenePtr planning_scene_; /**< Planning scene that maintains state of the world including the robot */
      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_; /**< Subscribes to the planning_scene_ topic */

      /**
       * Executive callback that executes the flower sequence planner
       */
      void execute(const manipulation_planning::PlanFlowerSequenceGoal::ConstPtr &goal);

      /**
       * Goal callback function for action server (prevents action server error that states no goal calkback has been registered)
       */
      void goal_clk();

      void flower_poses_clk(const manipulation_common::FlowerMap::ConstPtr &poses);
      bool evaluate_plan(const robot_state::RobotState &start_state, moveit::planning_interface::MoveGroupInterface &group, int &cost);
      geometry_msgs::Pose get_desired_previsit_ee_pose(const geometry_msgs::Pose flower_pose);

  };
}

#endif // PLAN_FLOWER_SEQUENCE_ACTION_HPP
