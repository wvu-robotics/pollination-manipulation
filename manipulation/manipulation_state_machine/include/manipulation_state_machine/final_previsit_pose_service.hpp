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
#ifndef FINAL_PREVISIT_POSE_SERVICE_HPP
#define FINAL_PREVISIT_POSE_SERVICE_HPP

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <manipulation_state_machine/FinalPrevisitPose.h>

#include <cmath> // trig functions
#define _USE_MATH_DEFINES // for math constants (M_PI)

namespace manipulation
{
  class FinalPrevisitPoseService
  {
    public:
      FinalPrevisitPoseService();
      ~FinalPrevisitPoseService(){};

    private:
      const double EE_OFFSET_FROM_FLOWER = -0.20; /**< Desired end effector offset position from a flower */

      ros::NodeHandle nh_; /**< ROS node handle */
      ros::ServiceServer srv_final_previsit_pose_;

      bool find_final_pose(manipulation_state_machine::FinalPrevisitPose::Request &req,
                      manipulation_state_machine::FinalPrevisitPose::Response &res);

      geometry_msgs::Pose get_desired_previsit_ee_pose(const geometry_msgs::Pose flower_pose);

  };
}

#endif //FINAL_PREVISIT_POSE_SERVICE_HPP
