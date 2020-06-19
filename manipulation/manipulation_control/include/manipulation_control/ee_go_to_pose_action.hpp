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
#ifndef EE_GO_TO_POSE_ACTION_SERVER_HPP
#define EE_GO_TO_POSE_ACTION_SERVER_HPP

#include <ros/ros.h>

// actionlib
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

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

// C++ standard libraries
#include <cmath> // trig functions

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

// Custom action/msgs
#include <manipulation_control/EEGoToPoseAction.h>
#include <manipulation_control/EECameraPose.h>
#include <manipulation_control/StartPollination.h>

namespace manipulation
{
  /**
   * An Action Server that plans a path for the end effector (EE/ee) to a particular pose
   */
  class EEGoToPoseAction
  {
    public:
      EEGoToPoseAction(std::string action_name, ros::NodeHandle &nh, moveit_visual_tools::MoveItVisualTools vt);
      ~EEGoToPoseAction(){};

    private:
      ros::NodeHandle nh_; /**< ROS node handle */
      std::string robot_type_;  /**< Needed for Kinova API & MoveIt for motion planning */
      std::string action_name_; /**< Stored action name */
      bool start_pollination_; /**< Flag that begins/ends manipulation pollination procedure */

      // Subscribers
      ros::Subscriber sub_start_pollination_; /**< Subscribes to topic /bramblebee/arm/start_pollintation that determines when to start/stop pollination procedure */
      // Kinova Subscribers
      ros::Subscriber sub_joint_state_; /**< Subscribes to the Kinova arm's JointState feedback from topic /j2n6s300_driver/out/joint_state */

      // Publishers
      ros::Publisher pub_cam_pose_; /**< Publishes EE camera's quaternion and xyz to /bramblebee/arm/camera_pose & publishes as action server's feedback */
      // MoveIt Publishers
      ros::Publisher pub_collision_object_; /**< Publishes to /collision_object which is added to the planning scene for obstacle avoidance by MoveIt */
      ros::Publisher pub_planning_scene_diff_; /**< Publishes to planning_scene, this is used by MoveIt to update its planning_scene */

      // action dependencies
      actionlib::SimpleActionServer<manipulation_control::EEGoToPoseAction> action_server_; /**< Action server for EE path planning to goal pose */
      manipulation_control::EEGoToPoseGoal goal_; /**< Pose goal */
      manipulation_control::EEGoToPoseFeedback feedback_; /**< Pose feedback*/
      manipulation_control::EEGoToPoseResult result_; /**< Pose result */

      //moveit
      moveit_visual_tools::MoveItVisualTools visual_tools_; /**< MoveIt visual tools used for visual aid. Used for visualizing projected pose estimates, planned move trajectories, etc */
      moveit::planning_interface::MoveGroupInterface *group_arm_; /**< Bramblebee's arm group used for planning and control in MoveIt */
      robot_model::RobotModelPtr robot_model_; /**< Bramblebee's arm kinematic model */
      robot_state::RobotStatePtr robot_state_; /**< Bramblebee's current state (pose) */
      moveit::planning_interface::MoveItErrorCode moveit_result_; /**< Result from attempt to move to a pose */
      planning_scene::PlanningScenePtr planning_scene_; /**< Planning scene that maintains state of the world including the robot */
      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_; /**< Subscribes to the planning_scene_ topic */
      moveit_msgs::PlanningScene planning_scene_msg_; /**< Message that is sent to add/remove objects from MoveIt's planning scene */
      moveit_msgs::CollisionObject collision_object_msg_; /**< Collision object that is used in adding/removing objects from MoveIt's planning scene */

      sensor_msgs::JointState current_joint_state_; /**< Current joint state is provided by Kinova that is used to find end effector pose */

      // ee === end effector === camera
      Eigen::Affine3d ee_state_; /**< Transformation matrix of end effector */
      Eigen::MatrixXd ee_jacobian_; /**< Jacobian matrix of end effector */
      tf2::Matrix3x3 rot_g2ee_; /**< Rotation matrix from global to end effector's reference frame (retrieved from ee_state_) */
      tf2::Quaternion quat_g2ee_; /**< Quaternion from global to end effector's reference frame (retrieved from ee_rot_g2ee_) */
      tf2::Vector3 ee_xyz_; /**< XYZ position of end effector (retrieved from ee_state_) */

      boost::mutex mutex_joint_state_; /**< used for synchronization */


      /**
       * Callback function for sub_start_pollination_ subscriber that determines when to begin/end pollination procedure
       */
      void start_pollination_clk(const std_msgs::Bool::ConstPtr &msg);

      /**
       * Executive callback that executes the EE's planned path to goal pose action
       */
      void execute(const manipulation_control::EEGoToPoseGoal::ConstPtr &goal);

      /**
       * Returns true if end effector motion reaches goal
       */
      bool evaluate_plan(moveit::planning_interface::MoveGroupInterface &group);

      /**
       * Clears objects from MoveIt's planning scene
       */
      void clear_workscene();

      /**
       * Adds AERB 222 development table
       */
      void add_dev_table();

      /**
       * Adds Bramblebee's top plate to the MoveIt's planning scene
       */
      void add_bramblebee_top_plate();

      /**
       * Adds Bramblebee's arch to the MoveIt's planning scene
       */
      void add_brambleebee_arch();

      /**
       * Adds Bramblebee's computer box to the MoveIt's planning scene
       * This includes the greenworks batteries (made the box extend deeper to include the batteries there is not a separate battery object)
       * TODO: low priority - add separate battery object
       */
      void add_brambleebee_computer_box();

      /**
       * Retrieves end effector pose and publishes it to topic /bramblebee/arm/camera_pose
       */
      void get_ee_pose();

      /**
       * Callback function for sub_joint_state_ subscriber to retrieve the kinova arm's current joint states
       */
      void get_current_state_clk(const sensor_msgs::JointStateConstPtr &msg);

      /**
       * Goal callback function for action server (prevents action server error that states no goal calkback has been registered)
       */
      void goal_clk();
  };
}

#endif // EE_GO_TO_POSE_ACTION_SERVER_HPP
