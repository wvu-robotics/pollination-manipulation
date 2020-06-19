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
#include <manipulation_state_machine/final_previsit_pose_service.hpp>

using namespace manipulation;

FinalPrevisitPoseService::FinalPrevisitPoseService()
{
  srv_final_previsit_pose_ = nh_.advertiseService("final_previsit_pose", &FinalPrevisitPoseService::find_final_pose, this);
}

bool FinalPrevisitPoseService::find_final_pose(manipulation_state_machine::FinalPrevisitPose::Request &_req,
                                          manipulation_state_machine::FinalPrevisitPose::Response &_res)
{
  _res.ee_previsit_pose = get_desired_previsit_ee_pose(_req.flower_pose);
  return true;
}

geometry_msgs::Pose FinalPrevisitPoseService::get_desired_previsit_ee_pose(const geometry_msgs::Pose _flower_pose)
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

  return des_ee_pose;
}
