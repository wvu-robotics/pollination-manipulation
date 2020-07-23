#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from manipulation_services.msg import controlAction, controlResult
from std_msgs.msg import Int32

import actionlib

class Run_Control_Service:
    def __init__(self):
        rospy.init_node('control',anonymous=True)
        rospy.loginfo("Fake_Control_Node_initialized")
        self.server = actionlib.SimpleActionServer('control', controlAction, self.control_handle, False)
        self.server.start()
        self.result = controlResult()
        rospy.spin()

    def control_handle(self,goal):
        rospy.loginfo("Data Received1")
        rospy.loginfo(goal.goal_pose)
        rospy.sleep(3.0)
        rospy.loginfo("Goal Reached")
        self.result.goal_reached = True
        self.server.set_succeeded(self.result)

def main():
    try:
        run_Control_service = Run_Control_Service()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
