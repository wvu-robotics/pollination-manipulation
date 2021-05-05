#!/usr/bin/env python

import rospy

from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from manipulation_control.msg import EEGoToPoseAction, EEGoToPoseGoal
from manipulation_vision.msg import ApproachFlowerAction, ApproachFlowerGoal, ApproachFlowerResult
import actionlib
import kinova_msgs.msg

import kinova_msgs.msg



import tf2_ros
from tf import transformations as t
import numpy as np
import math
import actionlib

class ApproachFlower:
    def __init__(self):
        rospy.init_node('approach_flower', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Approach Flower Node Initialized")

        self.server = actionlib.SimpleActionServer('/approach_flower', ApproachFlowerAction, self.approach_handle, False)
        self.result = ApproachFlowerResult()
        self.server.start()
        rospy.spin()

    def approach_handle(self, goal):

        rospy.loginfo("Approach Flower procedure started Started")

        _start_polination_position = rospy.wait_for_message("/j2n6s300_driver/out/joint_command", kinova_msgs.msg.JointAngles)
        # Process the goal to the Flower
        pos_flower = np.array([goal.flower_pose.position.x, goal.flower_pose.position.y, goal.flower_pose.position.z])
        #new_offset = np.multiply(np.array([0.13,0.13,0.13]),np.array([goal.flower_vector.vector.x,goal.flower_vector.vector.y,goal.flower_vector.vector.z]))
        #new_offset = np.array([0.13,0.13,0.13])
        des_pos = pos_flower#+new_offset
        _send_goal = PoseStamped()
        _send_goal.pose.position.x= des_pos[0]+0.035
        _send_goal.pose.position.y= des_pos[1]-0.1367
        _send_goal.pose.position.z= des_pos[2]+0.098
        _send_goal.pose.orientation = goal.flower_pose.orientation

        print("Send goal:{}".format(_send_goal))

        _control_client = actionlib.SimpleActionClient('ee_go_to_pose', EEGoToPoseAction)

        _control_client.wait_for_server()
        _control_goal = EEGoToPoseGoal(goal_pose=_send_goal)
        _control_client.send_goal(_control_goal)
        _control_client.wait_for_result()
        _control_result=_control_client.get_result()
        # rospy.loginfo(_control_result)

        if _control_result.goal_reached==True:
            self.result.flower_approached   = True
        else:
            self.result.flower_approached = False


        client = actionlib.SimpleActionClient( "/j2n6s300_driver/joints_action/joint_angles",kinova_msgs.msg.ArmJointAnglesAction)
        rospy.loginfo("Going back to previous position")
        goal = kinova_msgs.msg.ArmJointAnglesGoal()
        goal.angles = _start_polination_position
        client.wait_for_server()
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(20.0)):
            result = client.get_result()

        self.server.set_succeeded(self.result)



    def shutdown(self):
        rospy.loginfo("Approach Flower node is shutdown")
        rospy.sleep(1)

def main():
    try:
        approachflower = ApproachFlower()

    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
