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
#include <manipulation_state_machine/find_previsit_poses_service.hpp>

using namespace manipulation;

FindPrevisitPosesService::FindPrevisitPosesService()
{
  srv_find_previsit_poses_ = nh_.advertiseService("find_previsit_poses", &FindPrevisitPosesService::find_poses, this);
}

bool FindPrevisitPosesService::find_poses(manipulation_state_machine::FindPrevisitPoses::Request &_req,
                                          manipulation_state_machine::FindPrevisitPoses::Response &_res)
{
  geometry_msgs::Pose pose = _req.ee_previsit_pose;
  _res.ee_poses.push_back(pose);

  tf2::Quaternion orig_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  orig_quat.normalize();

  double change_deg = 15; // degrees
  double crad = change_deg * M_PI/180;

  tf2::Quaternion rot_quat;
  rot_quat.setRPY(crad, 0, 0);
  tf2::Quaternion new_quat = rot_quat * orig_quat;
  new_quat.normalize();

  tf2::convert(new_quat, pose.orientation);

  pose.position.z = pose.position.z + 0.055;
  _res.ee_poses.push_back(pose);

  pose = _req.ee_previsit_pose;

  rot_quat.setRPY(-crad, 0, -crad);
  new_quat = rot_quat * orig_quat;
  new_quat.normalize();

  tf2::convert(new_quat, pose.orientation);

  pose.position.x = pose.position.x + 0.055;
  pose.position.z = pose.position.z - 0.055;
  _res.ee_poses.push_back(pose);


  pose = _req.ee_previsit_pose;
  rot_quat.setRPY(-crad, 0, crad);
  new_quat = rot_quat * orig_quat;
  new_quat.normalize();

  tf2::convert(new_quat, pose.orientation);

  pose.position.x = pose.position.x - 0.055;
  pose.position.z = pose.position.z - 0.055;
  _res.ee_poses.push_back(pose);

  _res.ee_poses.push_back(_req.ee_previsit_pose);

  return true;
}
