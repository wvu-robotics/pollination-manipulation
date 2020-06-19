#include <ros/ros.h>

#include <manipulation_control/EEGoToPoseAction.h>
#include <actionlib/client/simple_action_client.h>

// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::vector<double> quat2eulerZYX(double x, double y, double z, double w);
std::vector<double> eulerZYX2quat(double rotx, double roty, double rotz);
void test_conversions();

int main(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "test_pose_flower_node");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<manipulation_control::EEGoToPoseAction> control_action_server("ee_go_to_pose", true);
  ROS_INFO("Waiting for action server...");
  control_action_server.waitForServer();
  ROS_INFO("Action server started");

  //test converations
  test_conversions();

  //define orientation using ZYX euler angles (i.e., R = Rx*Ry*Rz)
  double rotx = -90.0*3.14159265/180.0;
  double roty =   0.0*3.14159265/180.0;
  double rotz = 180.0*3.14159265/180.0;

  //convert to euler angles to quaternion (quat required for action server)
  std::vector<double> q = eulerZYX2quat(rotx, roty, rotz);
  tf2::Quaternion quat(q[0],
                       q[1],
                       q[2],
                       q[3]);
  quat.normalize();

  //define position in arm frame (x left arm, y behind arm, z up)
  double x =  0.00;
  double y = -0.30;
  double z =  0.50;

  // set goal pose
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.stamp = ros::Time::now();
  goal_pose.header.frame_id = "j2n6s300_link_base";

  goal_pose.pose.position.x =  0.00;
  goal_pose.pose.position.y = -0.30;
  goal_pose.pose.position.z =  0.50;

  tf2::convert(quat, goal_pose.pose.orientation);

  //send goal pose
  manipulation_control::EEGoToPoseGoal goal;
  goal.goal_pose = goal_pose;
  control_action_server.sendGoal(goal);

  //wait to reach goal
  ROS_INFO("Planning path to goal...");
  bool finished_goal = control_action_server.waitForResult();
  if (finished_goal && control_action_server.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Finished planning to goal pose");
  }
  else
  {
    ROS_ERROR("Was not able to plan path to goal pose");
  }

  manipulation_control::EEGoToPoseResult::ConstPtr result = control_action_server.getResult();
  if (result->goal_reached)
  {
    ROS_INFO("Goal was reached!");
  }
  else
  {
    ROS_ERROR("Goal was not reached.");
  }

  return 0;
}

std::vector<double> quat2eulerZYX(double x, double y, double z, double w)
{
  double q0 = w;
  double q1 = x;
  double q2 = y;
  double q3 = z;
  double rotx = std::atan2( 2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2) );
  double roty = std::asin( 2*(q0*q2 - q3*q1) );
  double rotz = std::atan2( 2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3) );

  std::vector<double> output;
  output.push_back(rotx);
  output.push_back(roty);
  output.push_back(rotx);
  return output;
};

//returns normalized quaternion where q = [qx, qy, qz, qw]
std::vector<double> eulerZYX2quat(double rotx, double roty, double rotz)
{
  double cz = std::cos(rotz*0.5);
  double sz = std::sin(rotz*0.5);
  double cy = std::cos(roty*0.5);
  double sy = std::sin(roty*0.5);
  double cx = std::cos(rotx*0.5);
  double sx = std::sin(rotx*0.5);

  double qw = cx*cy*cz + sx*sy*sz;
  double qx = sx*cy*cz - cx*sy*sz;
  double qy = cx*sy*cz + sx*cy*sz;
  double qz = cx*cy*sz - sx*cy*cz;

  double normq = std::sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
  qw = qw/normq;
  qx = qx/normq;
  qy = qy/normq;
  qz = qz/normq;

  std::vector<double> output;
  output.push_back(qx);
  output.push_back(qy);
  output.push_back(qz);
  output.push_back(qw);
  return output;
};

void test_conversions()
{
  //test quat2eulerZYX
  double ex_qw =  0.014; //w
  double ex_qx =  0.024; //x
  double ex_qy =  0.847; //y
  double ex_qz = -0.530; //x
  std::vector<double> example_euler = quat2eulerZYX(ex_qx,
                                                    ex_qy,
                                                    ex_qz,
                                                    ex_qw);
  ROS_INFO("rotx, roty, rotz = %f, %f, %f", example_euler[0],
                                            example_euler[1],
                                            example_euler[2]);

  //test eulerZYX2quat
  double ex_rotx = example_euler[0];
  double ex_roty = example_euler[1];
  double ex_rotz = example_euler[2];
  std::vector<double> example_quat = eulerZYX2quat(ex_rotx,
                                                   ex_roty,
                                                   ex_rotz);

  //output tests
  ROS_INFO("test (in): x, y, z, w = %f, %f, %f, %f", ex_qx,
                                                     ex_qy,
                                                     ex_qz,
                                                     ex_qw);

  ROS_INFO("test (in): rotx, roty, rotz = %f, %f, %f", ex_rotx,
                                                       ex_roty,
                                                       ex_rotz);

  ROS_INFO("test (out): x, y, z, w = %f, %f, %f, %f", example_quat[0],
                                                      example_quat[1],
                                                      example_quat[2],
                                                      example_quat[3]);

  ROS_INFO("test (out): rotx, roty, rotz = %f, %f, %f", example_euler[0],
                                                        example_euler[1],
                                                        example_euler[2]);
};
