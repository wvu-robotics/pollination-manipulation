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
        self.tfBuffer = tf2_ros.Buffer()
        self.pose = PoseStamped()
        rospy.Subscriber("/j2n6s300_driver/out/tool_pose", PoseStamped, self.end_effector_kinova_position)
        rospy.Subscriber(topic_address, kinova_msgs.msg.JointAngles, current_pos)

        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
        #self.start_pollination.data = True
        rospy.sleep(2)
        self.rate = rospy.Rate(1.0)
        self.start()

    def start(self):
        while not rospy.is_shutdown():
            #try:
            #    _trans = self.tfBuffer.lookup_transform('j2n6s300_link_base','j2n6s300_end_effector', rospy.Time(0))
            #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #    self.rate.sleep()
            #    rospy.logerr("Transform was not available")
            #    continue
            #rospy.loginfo("transform1:")
            #rospy.loginfo(_trans.transform)
            #homog_matrix = t.quaternion_matrix([_trans.transform.rotation.x,_trans.transform.rotation.y,_trans.transform.rotation.x,_trans.transform.rotation.z])
            #homog_matrix = np.copy(homog_matrix)+np.array([[0,0,0,_trans.transform.translation.x],[0,0,0,_trans.transform.translation.y],[0,0,0,_trans.transform.translation.z],[0,0,0,0]])
            #rospy.loginfo(homog_matrix)
            #transform_matrix = np.matmul(np.array([[1.0000000, 0.0000000,  0.0000000, 0],[0.0000000,  1.0000000,  0.0000000, 0],[0.0000000,  0.0000000,  1.0000000, 0],[0, 0 ,0 ,1]]),homog_matrix)
            ##transform_matrix = np.linalg.inv(transform_matrix)
            #rospy.loginfo(transform_matrix)
            #quat = t.quaternion_from_matrix(transform_matrix)
#           # rospy.loginfo(quat)
            #_send_goal = PoseStamped()

            #_send_goal.pose.position.x = transform_matrix[0][3]
            #_send_goal.pose.position.y = transform_matrix[1][3]
            #_send_goal.pose.position.z = transform_matrix[2][3]
#           # _send_goal.pose.orientation.x = quat[0]
#           # _send_goal.pose.orientation.y = quat[1]
#           # _send_goal.pose.orientation.z = quat[2]
#           # _send_goal.pose.orientation.w = quat[3]
##
            #_send_goal.pose.orientation.x = _trans.transform.rotation.x
            #_send_goal.pose.orientation.y = _trans.transform.rotation.y
            #_send_goal.pose.orientation.z = _trans.transform.rotation.z
            #_send_goal.pose.orientation.w = _trans.transform.rotation.w


            rospy.loginfo(_send_goal)
#            rospy.loginfo(self.pose)
#
            _control_client = actionlib.SimpleActionClient('ee_go_to_pose', EEGoToPoseAction)
            _control_client.wait_for_server()
            _control_goal = EEGoToPoseGoal(goal_pose=_send_goal)
            _control_client.send_goal(_control_goal)
            _control_client.wait_for_result()
            _control_result=_control_client.get_result()
            rospy.loginfo(_control_result)
            self.rate.sleep()

    def end_effector_kinova_position(self,data):
        self.pose = data


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
