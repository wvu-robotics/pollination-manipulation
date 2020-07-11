#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

from manipulation_mapping.msg import BuildMapAction, BuildMapGoal , BuildMapResult
from manipulation_common.msg import PlanFlowerSequenceAction, PlanFlowerSequenceGoal
from manipulation_control.msg import EEGoToPoseAction, EEGoToPoseGoal
from manipulation_vision.msg import ApproachFlowerAction, ApproachFlowerGoal
from manipulation_pollinator.srv import PollinateFlower
from manipulation_common.srv import SearchForFlowers
from manipulation_state_machine.srv import FindPrevisitPoses
from manipulation_state_machine.srv import FinalPrevisitPose
import kinova_msgs.msg

from tf import transformations as t
import numpy as np
import math
import actionlib

from manipulation_common.srv import SearchForFlowers



import tf2_ros
from tf import transformations as t
import numpy as np
import math
import actionlib

#to do: add octomap

#Listing useful poses for camera
#Above right of the tree: 
#Trans = [-.285,.268,.733]
#Rot = [.827,.227,-.456,.238]
#Above Left of the tree:
#Trans = [-.117,-.280,.742]
#Rot = [.781,-0.074,-.512,-.349]
#Bottom Left of the tree:
#Trans = [-.396,-.016, .300]
#Rot = [-0.382, .477, .397,.686]
#Bottom Right of the tree:
#Trans = [-.313,.325,.322]
#Rot = [-.589,-.243,.650,-.415]
#Middle stright on frame:
#Trans = [.002,.034,.470]
#Rot = [-.576,.323,.681,-.315]

class BuildMap:
    def __init__(self):
    	rospy.init_node('build_map', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("BuildMap node started")

        self.current_pose = kinova_msgs.msg.KinovaPose()
        self._control_client = actionlib.SimpleActionClient('ee_go_to_pose', EEGoToPoseAction)
        #rospy.Subscriber("/j2n6s300_driver/out/joint_command", kinova_msgs.msg.JointAngles, self.current_joint_pose_kinova_degree)
        self.server = actionlib.SimpleActionServer('/build_map', BuildMapAction, self.mapping_handle, False)
        self.result = BuildMapResult()
        self.server.start()
        rospy.spin()

    def send_pose(self,pose_goal):
        _send_goal = PoseStamped()
        #put in a rotation transform here since currently having issue with global reference, eventually fix that issue and remove
        _send_goal.pose = pose_goal
        _control_goal = EEGoToPoseGoal(goal_pose=_send_goal)
        self._control_client.send_goal(_control_goal)
        self._control_client.wait_for_result()
        _control_result=self._control_client.get_result()
        return _control_result.goal_reached

    def mapping_handle(self, goal):
        #rospy.loginfo(self.joint)
        rospy.loginfo("Start Mapping")
        _pose_goal = Pose()
        
        #goal = kinova_msgs.msg.ArmJointAnglesGoal()

#Initial (1
        rospy.loginfo("position 1: Top Right of Tree")
        # goal.angles.joint1 =  270.0
        # goal.angles.joint2 =  120.0
        # goal.angles.joint3 =  30.0
        # goal.angles.joint4 =  120.0
        # goal.angles.joint5 =  0.0
        # goal.angles.joint6 =  -90.0
        # goal.angles.joint7 =  0


        _pose_goal.position.x = 0.285
        _pose_goal.position.y = -0.268
        _pose_goal.position.z = 0.733
        _pose_goal.orientation.x = -0.227
        _pose_goal.orientation.y = 0.827
        _pose_goal.orientation.z = 0.238
        _pose_goal.orientation.w = 0.456
        self.send_pose(_pose_goal)
        # client.wait_for_server()
        # client.send_goal(goal)
        # if client.wait_for_result(rospy.Duration(20.0)):
        #     result = client.get_result()
        # else:
        #     print('        the joint angle action timed-out')
        #     client.cancel_all_goals()
        self.search_for_flowers()

#(2
        rospy.loginfo("position 2: Top Left of Tree")
        # goal.angles.joint1 =  310.0
        # goal.angles.joint2 =  150.0
        # goal.angles.joint3 =  160.0
        # goal.angles.joint4 =  220
        # goal.angles.joint5 =  -170.0
        # goal.angles.joint6 =  -75.0
        # goal.angles.joint7 =  0.0
        
        _pose_goal.position.x = -0.085
        _pose_goal.position.y = 0.125
        _pose_goal.position.z = 0.784
        _pose_goal.orientation.x = 0.310
        _pose_goal.orientation.y = 0.721
        _pose_goal.orientation.z = -0.116
        _pose_goal.orientation.w = 0.609
        self.send_pose(_pose_goal)
        self.search_for_flowers()

#(3
        rospy.loginfo("position 3:Bottom Left of Tree")
        # goal.angles.joint1= 370
        # goal.angles.joint2= 180
        # goal.angles.joint3= 75
        # goal.angles.joint4= 180
        # goal.angles.joint5= -140
        # goal.angles.joint6= 15.0
        # goal.angles.joint7= 0.0
        _pose_goal.position.x = 0
        _pose_goal.position.y = 0.236
        _pose_goal.position.z = 0.396
        _pose_goal.orientation.x = 0.297
        _pose_goal.orientation.y = 0.658
        _pose_goal.orientation.z = -0.146
        _pose_goal.orientation.w = 0.676
        self.send_pose(_pose_goal)

        self.search_for_flowers()

#(4
        rospy.loginfo("position 4:Bottom Right of Tree")
        # goal.angles.joint1= 370
        # goal.angles.joint2= 230
        # goal.angles.joint3= 80
        # goal.angles.joint4= 170
        # goal.angles.joint5= -120
        # goal.angles.joint6= 55
        # goal.angles.joint7= 0.0
        _pose_goal.position.x = 0.168
        _pose_goal.position.y = -0.487
        _pose_goal.position.z = 0.300
        _pose_goal.orientation.x = -0.076
        _pose_goal.orientation.y = 0.727
        _pose_goal.orientation.z = 0.429
        _pose_goal.orientation.w = 0.532
        self.send_pose(_pose_goal)
        # client.wait_for_server()
        # client.send_goal(goal)
        # if client.wait_for_result(rospy.Duration(20.0)):
        #     result = client.get_result()
        # else:
        #     print('        the joint angle action timed-out')
        #     client.cancel_all_goals()
        self.search_for_flowers()

# #(Go to thebeggining again just to change sides)
#         goal.angles.joint1 =  270.0
#         goal.angles.joint2 =  120.0
#         goal.angles.joint3 =  80.0
#         goal.angles.joint4 =  100.0
#         goal.angles.joint5 =  0.0
#         goal.angles.joint6 =  -90.0
#         goal.angles.joint7 =  0

#         # client.wait_for_server()
#         # client.send_goal(goal)
#         # if client.wait_for_result(rospy.Duration(20.0)):
#         #     result = client.get_result()
#         # else:
#         #     print('        the joint angle action timed-out')
#         #     client.cancel_all_goals()

#(5
        rospy.loginfo("position 5:Middle of Tree")
        # goal.angles.joint1= 170
        # goal.angles.joint2= 230
        # goal.angles.joint3= 90
        # goal.angles.joint4= 0
        # goal.angles.joint5= 130
        # goal.angles.joint6= -170
        # goal.angles.joint7= 0.0
        _pose_goal.position.x = 0.017
        _pose_goal.position.y = -0.041
        _pose_goal.position.z = 0.549
        _pose_goal.orientation.x = -0.056
        _pose_goal.orientation.y = 0.731
        _pose_goal.orientation.z = 0.082
        _pose_goal.orientation.w = 0.676
        self.send_pose(_pose_goal)
        # client.wait_for_server()
        # client.send_goal(goal)
        # if client.wait_for_result(rospy.Duration(20.0)):
        #     result = client.get_result()
        # else:
        #     print('        the joint angle action timed-out')
        #     client.cancel_all_goals()
        self.search_for_flowers()
#Unused atm
# #(6
#         rospy.loginfo("position 6")
#         goal.angles.joint1= 170
#         goal.angles.joint2= 180
#         goal.angles.joint3= 75
#         goal.angles.joint4= -10
#         goal.angles.joint5= 160
#         goal.angles.joint6= -155
#         goal.angles.joint7= 0.0

#         # client.wait_for_server()
#         # client.send_goal(goal)
#         # if client.wait_for_result(rospy.Duration(20.0)):
#         #     result = client.get_result()
#         # else:
#         #     print('the joint angle action timed-out')
#         #     client.cancel_all_goals()
#         # self.search_for_flowers()

# # (7
#         rospy.loginfo("position 7")

#         goal.angles.joint1 =  270.0
#         goal.angles.joint2 =  120.0
#         goal.angles.joint3 =  60.0
#         goal.angles.joint4 =  -10.0
#         goal.angles.joint5 =  50.0
#         goal.angles.joint6 =  20.0
#         goal.angles.joint7 =  0
#         # client.wait_for_server()
#         # client.send_goal(goal)
#         # if client.wait_for_result(rospy.Duration(20.0)):
#         #     result = client.get_result()
#         # else:
#         #     print('        the joint angle action timed-out')
#         #     client.cancel_all_goals()
#         # self.search_for_flowers()

        #completed mapping
        self.result.mapping_completed = True
        self.server.set_succeeded(self.result)

    def current_joint_pose_kinova_degree(self,data):
        self.joint = data


    def search_for_flowers(self):
        rospy.loginfo("Call search_for_flowers Service")
        #rospy.wait_for_service('searchFF')
        # time.sleep(1)
        # return
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
