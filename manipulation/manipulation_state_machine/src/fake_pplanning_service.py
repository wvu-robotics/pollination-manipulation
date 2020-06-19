#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from manipulation_services.msg import planningAction, planningResult
from std_msgs.msg import Int16

import actionlib

class Run_Planning_Server:
    def __init__(self):
        rospy.init_node('planning',anonymous=True)
        rospy.loginfo("Fake_Planning_Node_initialized")
        self.server = actionlib.SimpleActionServer('planning', planningAction, self.planning_handle, False)
        self.server.start()
        self.result = planningResult()
        rospy.spin()

    def planning_handle(self,goal):
        rospy.loginfo("Planning Finished")
        rospy.sleep(3.0)
        self.result.poses_order = [2,1,3]
        self.server.set_succeeded(self.result)

def main():
    try:
        run_planning_server = Run_Planning_Server()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
