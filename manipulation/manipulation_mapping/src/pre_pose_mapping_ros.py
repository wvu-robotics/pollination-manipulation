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
import kinova_msgs.msg
from manipulation_mapping.msg import BuildMapAction, BuildMapGoal, BuildMapResult
import actionlib

from manipulation_common.srv import SearchForFlowers



import tf2_ros
from tf import transformations as t
import numpy as np
import math
import actionlib

#to do: add octomap

class BuildMap:
    def __init__(self):
    	rospy.init_node('build_map', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("BuildMap node started")

        self.current_pose = kinova_msgs.msg.KinovaPose()
        rospy.Subscriber("/j2n6s300_driver/out/joint_command", kinova_msgs.msg.JointAngles, self.current_joint_pose_kinova_degree)
        self.server = actionlib.SimpleActionServer('/build_map', BuildMapAction, self.mapping_handle, False)
        self.result = BuildMapResult()
        self.server.start()
        rospy.spin()



    def mapping_handle(self, goal):
        rospy.loginfo(self.joint)
        rospy.loginfo("Start Mapping")
        client = actionlib.SimpleActionClient( "/j2n6s300_driver/joints_action/joint_angles",kinova_msgs.msg.ArmJointAnglesAction)
        goal = kinova_msgs.msg.ArmJointAnglesGoal()

#Initial (1
        rospy.loginfo("position 1")
        goal.angles.joint1 =  270.0
        goal.angles.joint2 =  120.0
        goal.angles.joint3 =  30.0
        goal.angles.joint4 =  120.0
        goal.angles.joint5 =  0.0
        goal.angles.joint6 =  -90.0
        goal.angles.joint7 =  0
        client.wait_for_server()
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(20.0)):
            result = client.get_result()
        else:
            print('        the joint angle action timed-out')
            client.cancel_all_goals()
        self.search_for_flowers()

#(2
        rospy.loginfo("position 2")
        goal.angles.joint1 =  310.0
        goal.angles.joint2 =  150.0
        goal.angles.joint3 =  160.0
        goal.angles.joint4 =  220
        goal.angles.joint5 =  -170.0
        goal.angles.joint6 =  -75.0
        goal.angles.joint7 =  0.0
        client.wait_for_server()
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(20.0)):
            result = client.get_result()
        else:
            print('        the joint angle action timed-out')
            client.cancel_all_goals()
        self.search_for_flowers()

#(3
        rospy.loginfo("position 3")
        goal.angles.joint1= 370
        goal.angles.joint2= 180
        goal.angles.joint3= 75
        goal.angles.joint4= 180
        goal.angles.joint5= -140
        goal.angles.joint6= 15.0
        goal.angles.joint7= 0.0

        client.wait_for_server()
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(20.0)):
            result = client.get_result()
        else:
            print('        the joint angle action timed-out')
            client.cancel_all_goals()
        self.search_for_flowers()

#(4
        rospy.loginfo("position 4")
        goal.angles.joint1= 370
        goal.angles.joint2= 230
        goal.angles.joint3= 80
        goal.angles.joint4= 170
        goal.angles.joint5= -120
        goal.angles.joint6= 55
        goal.angles.joint7= 0.0

        client.wait_for_server()
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(20.0)):
            result = client.get_result()
        else:
            print('        the joint angle action timed-out')
            client.cancel_all_goals()
        self.search_for_flowers()

#(Go to thebeggining again just to change sides)
        goal.angles.joint1 =  270.0
        goal.angles.joint2 =  120.0
        goal.angles.joint3 =  80.0
        goal.angles.joint4 =  100.0
        goal.angles.joint5 =  0.0
        goal.angles.joint6 =  -90.0
        goal.angles.joint7 =  0

        client.wait_for_server()
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(20.0)):
            result = client.get_result()
        else:
            print('        the joint angle action timed-out')
            client.cancel_all_goals()

#(5
        rospy.loginfo("position 5")
        goal.angles.joint1= 170
        goal.angles.joint2= 230
        goal.angles.joint3= 90
        goal.angles.joint4= 0
        goal.angles.joint5= 130
        goal.angles.joint6= -170
        goal.angles.joint7= 0.0

        client.wait_for_server()
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(20.0)):
            result = client.get_result()
        else:
            print('        the joint angle action timed-out')
            client.cancel_all_goals()
        self.search_for_flowers()

#(6
        rospy.loginfo("position 6")
        goal.angles.joint1= 170
        goal.angles.joint2= 180
        goal.angles.joint3= 75
        goal.angles.joint4= -10
        goal.angles.joint5= 160
        goal.angles.joint6= -155
        goal.angles.joint7= 0.0

        client.wait_for_server()
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(20.0)):
            result = client.get_result()
        else:
            print('the joint angle action timed-out')
            client.cancel_all_goals()
        self.search_for_flowers()

# (7
        rospy.loginfo("position 7")

        goal.angles.joint1 =  270.0
        goal.angles.joint2 =  120.0
        goal.angles.joint3 =  60.0
        goal.angles.joint4 =  -10.0
        goal.angles.joint5 =  50.0
        goal.angles.joint6 =  20.0
        goal.angles.joint7 =  0
        client.wait_for_server()
        client.send_goal(goal)
        if client.wait_for_result(rospy.Duration(20.0)):
            result = client.get_result()
        else:
            print('        the joint angle action timed-out')
            client.cancel_all_goals()
        self.search_for_flowers()

        #completed mapping
        self.result.mapping_completed = True
        self.server.set_succeeded(self.result)

    def current_joint_pose_kinova_degree(self,data):
        self.joint = data


    def search_for_flowers(self):
        rospy.loginfo("Call search_for_flowers Service")
        rospy.wait_for_service('searchFF')
        start_search_for_flowers=rospy.ServiceProxy('searchFF', SearchForFlowers)
        try:
            search_for_flowers_status = start_search_for_flowers()
            rospy.loginfo(search_for_flowers_status)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        #Add some exception
        return


    def shutdown(self):
        rospy.loginfo("Mapping node is shutdown")
        rospy.sleep(1)

def main():
    try:
        buildmap = BuildMap()

    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
