#!/usr/bin/env python

import rospy
from manipulation_control.msg import EEGoToPoseAction, EEGoToPoseGoal

from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import kinova_msgs.msg


import tf2_ros
from tf import transformations as t
import numpy as np
import math
import actionlib


class Unwind:
    def __init__(self):
    	rospy.init_node('unwind', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Unwind node started")
        rospy.Subscriber("/j2n6s300_driver/out/joint_command", kinova_msgs.msg.JointAngles, self.current_joint_pose_kinova_degree)

        rospy.sleep(2)
        self.rate = rospy.Rate(1.0)
        self.start()

    def start(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.joint)
            client = actionlib.SimpleActionClient( "/j2n6s300_driver/joints_action/joint_angles",kinova_msgs.msg.ArmJointAnglesAction)
            goal = kinova_msgs.msg.ArmJointAnglesGoal()

#Initial (1)
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
            rospy.loginfo("???")

            rospy.sleep(5)

#(2)
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
            rospy.loginfo("???")

            rospy.sleep(5)

##---
##
#
#(3)
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
            rospy.loginfo("???")

            rospy.sleep(5)



#(4)
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
            rospy.loginfo("???")

            rospy.sleep(5)
#
#(Go to the beggining again just to change sides)
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
            rospy.loginfo("???")

#(5)
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
            rospy.loginfo("???")

            rospy.sleep(5)

#(6)
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
                print('        the joint angle action timed-out')
                client.cancel_all_goals()
            rospy.loginfo("???")

            rospy.sleep(5)

# (7)
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
            rospy.loginfo("???")

            rospy.sleep(5)



#

    def current_joint_pose_kinova_degree(self,data):
        self.joint = data


    def individual_flower_pollination_procedure(self):
        rospy.loginfo("Pollinating Flower: %f",self.flowers_order[self.number_flowers_pollinated])
        for _flower in self.flowers.map:
            if _flower.id ==  self.flowers_order[self.number_flowers_pollinated]:
                rospy.loginfo(_flower.pose)

        _control_client = actionlib.SimpleActionClient('ee_go_to_pose', EEGoToPoseAction)
        self.current_flower = self.flowers_with_offset[self.number_flowers_pollinated]
        #self.current_flower.position.x = -0.0612318253559
        #self.current_flower.position.y = -0.562394637376
        #self.current_flower.position.z = 0.445874788602

        #self.current_flower.orientation.x = -1.96422831761e-05
        #self.current_flower.orientation.y = 0.707111340811
        #self.current_flower.orientation.z = -0.707102221251
        #self.current_flower.orientation.w =  3.5750321417e-06

        # rospy.loginfo("first current pose")
        # rospy.loginfo(self.current_flower)

        rospy.wait_for_service('find_previsit_poses')
        _find_previsit_poses=rospy.ServiceProxy('find_previsit_poses', FindPrevisitPoses)
        try:
            _previsit_poses = _find_previsit_poses(self.current_flower)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        # rospy.loginfo(_previsit_poses.ee_poses)

        #Move to the offset pose to better estimate flower position
        for _goal in _previsit_poses.ee_poses:
            _i=1
            _control_client.wait_for_server()
            _send_goal = PoseStamped()
            _send_goal.pose = _goal
            # rospy.loginfo(_send_goal)
            _control_goal = EEGoToPoseGoal(goal_pose=_send_goal)
            _control_client.send_goal(_control_goal)
            _control_client.wait_for_result()
            _control_result=_control_client.get_result()
            # if (_i==1 or _i==5)and _control_result.goal_reached==False:
            if _control_result.goal_reached==False:
                rospy.logerr("Skipping flower")
                self.flowers_not_pollinated_due_error.append(self.flowers_order[self.number_flowers_pollinated])
#                self.flowers_order = self.flowers_order.append(self.flowers_order[self.number_flowers_pollinated])
#                self.flowers_with_offset = self.flowers_with_offset.append(self.flowers_with_offset[self.number_flowers_pollinated])
                self.number_flowers_pollinated = self.number_flowers_pollinated +1
                if self.number_flowers_pollinated < len(self.flowers_order):
                   rospy.loginfo(self.number_flowers_pollinated)
                   self.individual_flower_pollination_procedure()
                else:
                   self.end()
            self.search_for_flowers()
            _i = _i+1
            # rospy.loginfo(_control_result)
            # rospy.loginfo("current Flower:")
            # rospy.loginfo(self.current_flower)
            #rospy.loginfo("Number of Flowers pollinated: %d", self.number_flowers_pollinated)
            #rospy.loginfo("Total Number of Flowers: %d", len(self.flowers_order))

        for _flower in self.flowers.map:
            if _flower.id ==  self.flowers_order[self.number_flowers_pollinated]:
                self.current_flower = _flower.pose


        rospy.loginfo("Final Previsit Pose")
        rospy.wait_for_service('final_previsit_pose')
        _final_previsit_pose=rospy.ServiceProxy('final_previsit_pose', FinalPrevisitPose)
        try:
            _final_pose = _final_previsit_pose(self.current_flower)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        # rospy.loginfo(_final_pose.ee_previsit_pose)

        #Go To position to start pollinating
        _send_goal = PoseStamped()
        _send_goal.pose = _final_pose.ee_previsit_pose
        _control_client.wait_for_server()
        _control_goal = EEGoToPoseGoal(goal_pose=_send_goal)
        _control_client.send_goal(_control_goal)
        _control_client.wait_for_result()
        _control_result=_control_client.get_result()
        # rospy.loginfo(_control_result)

        if _control_result.goal_reached==True:
            self.visual_servoing()
        else:
            self.flowers_not_pollinated_due_error.append(self.flowers_order[self.number_flowers_pollinated])
            self.number_flowers_pollinated = self.number_flowers_pollinated +1
            if self.number_flowers_pollinated < len(self.flowers_order):
               rospy.loginfo(self.number_flowers_pollinated)
               self.individual_flower_pollination_procedure()
            else:
               self.end()


    def search_for_flowers(self):
        rospy.loginfo("Call search_for_flowers Service")
        rospy.wait_for_service('search')
        start_search_for_flowers=rospy.ServiceProxy('search', SearchForFlowers)
        try:
            search_for_flowers_status = start_search_for_flowers()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        #Add some exception
        return

    def visual_servoing(self):
        rospy.loginfo("Visual_servoing")
        rospy.loginfo("Visual Servoing Flower: %d",self.flowers_order[self.number_flowers_pollinated])
        _visual_servoing_client = actionlib.SimpleActionClient('approach_flower', ApproachFlowerAction)
        _visual_servoing_client.wait_for_server()
        _visual_servoing_goal = ApproachFlowerGoal(flower_id=self.flowers_order[self.number_flowers_pollinated])
        _visual_servoing_client.send_goal(_visual_servoing_goal)
        rospy.sleep(1)
        _visual_servoing_client.wait_for_result()
        _visual_servoing_result=_visual_servoing_client.get_result().flower_approached
        rospy.loginfo("Visual servoing result: %r", _visual_servoing_result)
        if _visual_servoing_result==True:
            self.pollinating()
        else:
            self.logerr("Visual Servoing was not completed")
            self.individual_flower_pollination_procedure()
        return

    def pollinating(self):
        rospy.loginfo("pollinating")
        # rospy.sleep(1)
        rospy.wait_for_service('pollinate_flower')
        start_pollinating=rospy.ServiceProxy('pollinate_flower', PollinateFlower)
        try:
           pollinating_status = start_pollinating()
        except rospy.ServiceException as exc:
           print("Service did not process request: " + str(exc))
        self.number_flowers_pollinated = self.number_flowers_pollinated +1
        if self.number_flowers_pollinated<len(self.flowers_order):
            self.individual_flower_pollination_procedure()
        else:
            self.end()
        return

    def end(self):
        rospy.loginfo("Pollination Procedures ended")
        self.number_flowers_pollinated = 0
        self.start_pollination.data = False
        self.end_pollination.data = True

        print("List of Flowers not pollinated due error on reaching planning goal: ",self.flowers_not_pollinated_due_error)
        self.flowers_not_pollinated_due_error = [] # Reset the flowers that were not able to reach the goal - Later create a routine to deal with this flowers)

        self.pollination_end_pub.publish(self.end_pollination)
        self.pub.publish(self.start_pollination)
        self.flowers_order = Int16()
        self.flowers = FlowerMap()
        self.flowers_with_offset = Pose()
        self.start()

    def shutdown(self):
        rospy.loginfo("Unwind node is shutdown")
        rospy.sleep(1)

def main():
    try:
        unwind = Unwind()

    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
