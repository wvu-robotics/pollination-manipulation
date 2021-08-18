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



#Positions as of 8/12, j2n6s300 in respect to base_link
#
#Straight On
# - Translation: [0.760, 0.050, 0.591]
# - Rotation: in Quaternion [0.698, 0.261, 0.653, 0.132]
#             in RPY (radian) [1.781, -1.004, 1.551]
#             in RPY (degree) [102.017, -57.539, 88.886]


#Top Left
# - Translation: [0.643, 0.261, 0.733]
# - Rotation: in Quaternion [0.845, -0.131, 0.437, 0.278]
#             in RPY (radian) [2.487, -0.946, 0.037]
#             in RPY (degree) [142.489, -54.230, 2.095]
#Top Right
# - Translation: [0.697, -0.130, 0.763]
# - Rotation: in Quaternion [0.880, 0.227, 0.379, -0.175]
#             in RPY (radian) [-2.936, -0.843, 0.413]
#             in RPY (degree) [-168.239, -48.325, 23.689]
#Bottom Right
# - Translation: [0.575, -0.125, 0.531]
# - Rotation: in Quaternion [-0.482, 0.389, -0.216, 0.755]
#             in RPY (radian) [-1.317, 0.388, -0.859]
#             in RPY (degree) [-75.443, 22.229, -49.222]
#Bottom Left
# - Translation: [0.569, 0.224, 0.471]
# - Rotation: in Quaternion [0.160, 0.569, -0.194, 0.783]
#             in RPY (radian) [0.098, 1.263, -0.413]
#             in RPY (degree) [5.634, 72.340, -23.668]
# 


#Retract position
# - Translation: [0.497, 0.029, 0.550]
# - Rotation: in Quaternion [-0.094, 0.860, -0.012, 0.501]
#             in RPY (radian) [-2.916, 1.035, -2.795]
#             in RPY (degree) [-167.072, 59.276, -160.138]


#(1
        rospy.loginfo("position 1:Middle of Tree")
        # # goal.angles.joint1= 170
        # # goal.angles.joint2= 230
        # # goal.angles.joint3= 90
        # # goal.angles.joint4= 0
        # # goal.angles.joint5= 130
        # # goal.angles.joint6= -170
        # # goal.angles.joint7= 0.0
        _pose_goal.position.x = 0.760
        _pose_goal.position.y = 0.050
        _pose_goal.position.z = 0.591
        _pose_goal.orientation.x = 0.698
        _pose_goal.orientation.y = 0.261
        _pose_goal.orientation.z = 0.653
        _pose_goal.orientation.w = 0.132
        self.send_pose(_pose_goal)
        # # client.wait_for_server()
        # # client.send_goal(goal)
        # # if client.wait_for_result(rospy.Duration(20.0)):
        # #     result = client.get_result()
        # # else:
        # #     print('        the joint angle action timed-out')
        # #     client.cancel_all_goals()
        self.search_for_flowers()

#Initial (2
        rospy.loginfo("position 2: Top Right of Tree")
        # goal.angles.joint1 =  270.0
        # goal.angles.joint2 =  120.0
        # goal.angles.joint3 =  30.0
        # goal.angles.joint4 =  120.0
        # goal.angles.joint5 =  0.0
        # goal.angles.joint6 =  -90.0
        # goal.angles.joint7 =  0


        _pose_goal.position.x = 0.697
        _pose_goal.position.y = -0.130
        _pose_goal.position.z = 0.763
        _pose_goal.orientation.x = 0.880
        _pose_goal.orientation.y = 0.227
        _pose_goal.orientation.z = 0.379
        _pose_goal.orientation.w = -0.175
        self.send_pose(_pose_goal)
        # client.wait_for_server()
        # client.send_goal(goal)
        # if client.wait_for_result(rospy.Duration(20.0)):
        #     result = client.get_result()
        # else:
        #     print('        the joint angle action timed-out')
        #     client.cancel_all_goals()
        self.search_for_flowers()

#(3
        rospy.loginfo("position 3: Top Left of Tree")
        # goal.angles.joint1 =  310.0
        # goal.angles.joint2 =  150.0
        # goal.angles.joint3 =  160.0
        # goal.angles.joint4 =  220
        # goal.angles.joint5 =  -170.0
        # goal.angles.joint6 =  -75.0
        # goal.angles.joint7 =  0.0
        
        _pose_goal.position.x = 0.643
        _pose_goal.position.y = 0.261
        _pose_goal.position.z = 0.733
        _pose_goal.orientation.x = 0.845
        _pose_goal.orientation.y = -0.131
        _pose_goal.orientation.z = 0.437
        _pose_goal.orientation.w = 0.278
        self.send_pose(_pose_goal)
        self.search_for_flowers()

#(4
        rospy.loginfo("position 4:Bottom Left of Tree")
        # goal.angles.joint1= 370
        # goal.angles.joint2= 180
        # goal.angles.joint3= 75
        # goal.angles.joint4= 180
        # goal.angles.joint5= -140
        # goal.angles.joint6= 15.0
        # goal.angles.joint7= 0.0
        _pose_goal.position.x = 0.569
        _pose_goal.position.y = 0.224
        _pose_goal.position.z = 0.471
        _pose_goal.orientation.x = 0.160
        _pose_goal.orientation.y = 0.569
        _pose_goal.orientation.z = -0.194
        _pose_goal.orientation.w = 0.783
        self.send_pose(_pose_goal)

        self.search_for_flowers()

#(5
        rospy.loginfo("position 5:Bottom Right of Tree")
        # goal.angles.joint1= 370
        # goal.angles.joint2= 230
        # goal.angles.joint3= 80
        # goal.angles.joint4= 170
        # goal.angles.joint5= -120
        # goal.angles.joint6= 55
        # goal.angles.joint7= 0.0
        _pose_goal.position.x = 0.575
        _pose_goal.position.y = -0.125
        _pose_goal.position.z = 0.531
        _pose_goal.orientation.x = -0.482
        _pose_goal.orientation.y = 0.389
        _pose_goal.orientation.z = -0.216
        _pose_goal.orientation.w = 0.755
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


#I think this one is wrong, need to redo


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
