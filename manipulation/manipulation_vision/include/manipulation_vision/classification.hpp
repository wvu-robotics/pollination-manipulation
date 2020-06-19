#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <manipulation_vision/ClassifyFlowers.h>
#include <manipulation_vision/ClassifyPose.h>

#include <Python.h>
#include <iostream>
#include <bits/stdc++.h>
#include <string>

namespace manipulation
{

class Classification {
  public:
    Classification();

    bool classifyImage(manipulation_vision::ClassifyFlowers::Request  &req, 
                       manipulation_vision::ClassifyFlowers::Response &res);

    bool classifyPose(manipulation_vision::ClassifyPose::Request  &req, 
                      manipulation_vision::ClassifyPose::Response &res);

    ros::NodeHandle    nh;
    ros::ServiceServer classificationServ;
    ros::ServiceServer poseServ;
  
  private:
    //add prive members and methods here


};

}