#include <ros/ros.h>
#include <ros/console.h>

#include <actionlib/client/simple_action_client.h>
#include <manipulation_planning/PlanFlowerSequenceAction.h>

int main(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "test_plan_flower_sequence");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<manipulation_planning::PlanFlowerSequenceAction> planning_action_server("plan_flower_sequence", true);
  ROS_INFO("Waiting for action server...");
  planning_action_server.waitForServer();
  ROS_INFO("Action server started");

  std::vector<int16_t> flower_ids = {2,3,4};

  manipulation_planning::PlanFlowerSequenceGoal goal;
  goal.flower_ids = flower_ids;
  planning_action_server.sendGoal(goal);

  ROS_INFO("Currently planning..");
  bool finished_goal = false;

  finished_goal = planning_action_server.waitForResult();

  if (finished_goal && planning_action_server.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Finished planning");
  }
  else
  {
    ROS_ERROR("Was not able to plan");
  }

  manipulation_planning::PlanFlowerSequenceResult::ConstPtr result = planning_action_server.getResult();

  std::vector<int16_t> sequence(result->sequence);

  for (int16_t id : sequence)
    std::cout << id << std::endl;

  return 0;
}
