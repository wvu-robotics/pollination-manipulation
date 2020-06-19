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
#ifndef APPROACH_FLOWER_ACTION_HPP
#define APPROACH_FLOWER_ACTION_HPP

#include <ros/ros.h>

// actionlib
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

// kinova
#include <kinova_msgs/JointVelocity.h>

// moveit
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

// custom msgs
#include <manipulation_vision/ApproachFlowerAction.h>
#include <manipulation_control/EECameraPose.h>
#include <aruco_markers/MarkerArray.h>
#include <aruco_markers/Marker.h>
#include <manipulation_common/FlowerMap.h>
#include <manipulation_common/Flower.h>
#include <manipulation_common/SearchForFlowers.h>

#include <Eigen/Dense> // Matrix & Vectors
#define _USE_MATH_DEFINES // for math constants (M_PI)
#include <cmath>

namespace manipulation
{
  /**
   *  An action server that uses visual servoing techniques to apprach a flower
   */
  class ApproachFlowerAction
  {
    public:
      ApproachFlowerAction(std::string action_name, ros::NodeHandle &nh, moveit_visual_tools::MoveItVisualTools vt);
      ~ApproachFlowerAction(){};

    private:
      const std::string ROBOT_TYPE = "j2n6s300";  /**< Needed for Kinova API & MoveIt to get ee pose */
      const double EE_LOST_SIGHT_OFFSET = 0.18; /**< The offset in meters where the camera can no longer view the flower */
      // const double EE_LOST_SIGHT_OFFSET = 0.215; /**< The offset in meters where the camera can no longer view the flower */
      const double EE_OFFSET_FROM_FLOWER = -0.20; /**< Desired end effector offset position from a flower */
      const double PARALLEL_DIST_THRESHOLD = 0.005; /**< Parallel distance threshold for when alignment with z axis */
      const double MAX_APPROACH_TIME = 25.0; /**< Maximum time to allow approaching of flower */
      const double VELOCITY_SCALE = 5.0; /**< Velocity scale for joints to move (e.g. 5deg/second)*/

      ros::NodeHandle nh_; /**< ROS node handle */
      std::string action_name_; /**< Stored action name */

      // Subscribers
      ros::Subscriber sub_joint_state_; /**< Subscribes to the Kinova arm's JointState feedback from topic /j2n6s300_driver/out/joint_state */
      ros::Subscriber sub_markers_; /**< Subscribes to detected markers topic /markers; TODO: Remove when CV for flowers working */
      ros::Subscriber sub_flower_poses_;

      // Publishers
      ros::Publisher pub_joint_vel_; /**< Publishes to Kinova topic /j2n6s300_driver/in/joint_velocity (Kinova moves it) */
      ros::Publisher pub_cam_pose_; /**< Publishes EE camera's quaternion and xyz to /bramblebee/arm/camera_pose & publishes as action server's feedback */

      // action dependencies
      actionlib::SimpleActionServer<manipulation_vision::ApproachFlowerAction> action_server_; /**< Action server for final approach towards a flower */
      manipulation_vision::ApproachFlowerGoal goal_; /**< Approach flower goal */
      manipulation_vision::ApproachFlowerFeedback feedback_; /**< Approach flower feedback*/
      manipulation_vision::ApproachFlowerResult result_; /**< Approach flower result */

      //moveit
      moveit_visual_tools::MoveItVisualTools visual_tools_; /**< MoveIt visual tools used for visual aid. Used for visualizing projected pose estimates, planned move trajectories, etc */
      robot_model::RobotModelPtr robot_model_; /**< Bramblebee's arm kinematic model */
      robot_state::RobotStatePtr robot_state_; /**< Bramblebee's current state (pose) */
      planning_scene::PlanningScenePtr planning_scene_; /**< Planning scene that maintains state of the world including the robot */
      planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_; /**< Subscribes to the planning_scene_ topic */
      moveit::planning_interface::MoveGroupInterface *group_arm_; /**< Bramblebee's arm group used for planning and control in MoveIt */
      boost::mutex mutex_joint_state_; /**< used for synchronization; prevents race conditions */
      sensor_msgs::JointState current_joint_state_; /**< Current joint state is provided by Kinova that is used to find end effector pose */

      // ee === end effector === camera
      Eigen::Affine3d ee_state_; /**< Transformation matrix of end effector */
      Eigen::MatrixXd ee_jacobian_; /**< Jacobian matrix of end effector */
      tf2::Matrix3x3 rot_g2ee_; /**< Rotation matrix from global to end effector's reference frame (retrieved from ee_state_) */
      tf2::Quaternion quat_g2ee_; /**< Quaternion from global to end effector's reference frame (retrieved from ee_rot_g2ee_) */
      tf2::Vector3 ee_xyz_; /**< XYZ position of end effector (retrieved from ee_state_) */
      tf2::Vector3 eep_offset_; /**< end effector pollinator (eep) offset (i.e. pollinator offset from end effector) in meters */

      std::vector<aruco_markers::Marker> current_detected_markers_; /**< Detected markers that are currently seen; TODO: update when CV for flowers working */
      std::vector<aruco_markers::Marker> prev_detected_markers_; /**< Detected markers that were previously seen before ee position > EE_LOST_SIGHT_OFFSET; TODO: update when CV for flowers working */

      geometry_msgs::Pose des_previsit_pose_;
      std::vector<manipulation_common::Flower> current_detected_flowers_;
      manipulation_common::Flower prev_detected_flower_;
      int flower_id_;

      // //search service
      // ros::ServiceClient                    searchForFlowersClient_;
      // manipulation_common::SearchForFlowers searchForFlowersSrv_;


      /**
       * Executive callback that executes the EE's planned path to goal pose action
       */
      void execute(const manipulation_vision::ApproachFlowerGoal::ConstPtr &goal);

      /**
       * Visual servoing towards the flower
       */
      bool approach_flower(const double start_timer);

      /**
       * Retrieves the desired previsit flower end effector pose in global coordinates
       */
      // geometry_msgs::Pose get_desired_previsit_ee_pose(const aruco_markers::Marker marker);
      geometry_msgs::Pose get_desired_previsit_ee_pose(const geometry_msgs::Pose flower_pose);

      /**
       * Goal callback function for action server (prevents action server error that states no goal calkback has been registered)
       */
      void goal_clk();

      /**
       * Callback function for sub_markers_ subscriber to retrieve currently detected markers
       * TODO: update when CV for flowers working
       */
      void get_detected_markers_clk(const aruco_markers::MarkerArray::ConstPtr &msg);

      /**
       * Callback function for sub_joint_state_ subscriber to retrieve the kinova arm's current joint states
       */
      void get_current_state_clk(const sensor_msgs::JointStateConstPtr &msg);

      /**
       * Retrieves end effector pose and publishes it to topic /bramblebee/arm/camera_pose
       */
      void get_ee_pose();

      void flower_poses_clk(const manipulation_common::FlowerMap::ConstPtr &poses);

  };
}

#endif // APPROACH_FLOWER_ACTION_HPP
