#!/usr/bin/env python

import rospy
from manipulation_mapping.msg import BuildMapAction, BuildMapGoal
from manipulation_common.msg import PlanFlowerSequenceAction, PlanFlowerSequenceGoal
from manipulation_control.msg import EEGoToPoseAction, EEGoToPoseGoal
from manipulation_vision.msg import ApproachFlowerAction, ApproachFlowerGoal
from manipulation_pollinator.srv import PollinateFlower
from manipulation_common.srv import SearchForFlowers
from manipulation_state_machine.srv import FindPrevisitPoses
from manipulation_state_machine.srv import FinalPrevisitPose
import kinova_msgs.msg



from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from manipulation_common.msg import FlowerMap
from manipulation_common.msg import Flower


from tf import transformations as t
import numpy as np
import math
import actionlib


#Define states

class Pollination_manipulation:
    def __init__(self):
    	rospy.init_node('pollination_manipulation', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Pollination manipulation State Machine node started")
        self.pollination_end_pub = rospy.Publisher("pollination_procedure_ended", Bool,queue_size=1)
        self.pub = rospy.Publisher("start_pollination_procedures", Bool, queue_size = 1)
        self.flowers = FlowerMap() ####CHANGE THE TYPE OF MESSAGE FOR THE SUBSCRIBER ########
        rospy.Subscriber("/flower_mapper/flower_map", FlowerMap, self.global_flower_poses)
        self.flowers_with_offset = Pose()
        self.flowers_order = Int16()
        self.number_flowers_pollinated = 0
        self.flowers_not_pollinated_due_error = []
        self.start_pollination = Bool()
        self.end_pollination = Bool()
        #self.start_pollination.data = True
        self.current_flower = Pose()
        self._control_client = actionlib.SimpleActionClient('ee_go_to_pose', EEGoToPoseAction)
        #set the robot to home position by Joint Control

        self.start()

    def global_flower_poses(self,data):
        self.flowers = data

    def start(self):
        while not rospy.is_shutdown():
            self.home_position()
            self.start_pollination = rospy.wait_for_message("start_pollination_procedures",Bool)
            self.end_pollination.data = False
            self.pollination_end_pub.publish(self.end_pollination)
            if self.start_pollination.data == True:
                self.mapping()
                #these shouldn't be needed
                #self.search_for_flowers() #delete
                #self.planning()           #delete


    def mapping(self):
        rospy.loginfo("Map_state")
        #Skipping map part
        #self.individual_flower_pollination_procedure()
        _mapping_client = actionlib.SimpleActionClient("/build_map", BuildMapAction)
        _mapping_client.wait_for_server()
        _map_goal = BuildMapGoal(n_points=150000)
        _mapping_client.send_goal(_map_goal)
        _mapping_client.wait_for_result()
        _map_result=_mapping_client.get_result()
        rospy.loginfo(_map_result)
        # raw_input("Press Enter to continue...")
        if _map_result.mapping_completed==True:
            self.planning()
        else:
            rospy.logerr("Mapping action was not completed")
            self.end()

    def planning(self):
        rospy.loginfo("Planning")
        _ids = []
        for _id in self.flowers.map:
            # if (_id.pose.position.y <= -0.7): # 70cm
                # _ids.append(_id.id)
            _ids.append(_id.id)
        rospy.loginfo(_ids)
        if not _ids:
            rospy.logerr("No Flower were detected")
            self.end()
        _planning_client = actionlib.SimpleActionClient('plan_flower_sequence', PlanFlowerSequenceAction)
        _planning_client.wait_for_server()
        _planning_goal = PlanFlowerSequenceGoal(flower_ids=_ids)
        rospy.loginfo("Sending planning goal")
        _planning_client.send_goal(_planning_goal)
        _planning_client.wait_for_result()
        self.flowers_order=_planning_client.get_result().sequence
        self.flowers_with_offset = _planning_client.get_result().ee_previsit_poses
        rospy.loginfo(self.flowers_order)
        self.individual_flower_pollination_procedure()
    
    def send_pose(self,pose_goal):
        _send_goal = PoseStamped()
        #put in a rotation transform here since currently having issue with global reference, eventually fix that issue and remove        
        _send_goal.pose = pose_goal
        _control_goal = EEGoToPoseGoal(goal_pose=_send_goal)
        self._control_client.send_goal(_control_goal)
        self._control_client.wait_for_result()
        _control_result=self._control_client.get_result()
        return _control_result.goal_reached


#new version
    def individual_flower_pollination_procedure(self):
        rospy.loginfo("Pollinating state")
        rospy.loginfo(self.flowers_order)
        rospy.loginfo(self.flowers_with_offset)
        i = 0
        for pose in self.flowers_with_offset:
            if self.send_pose(pose) :
                rospy.loginfo("Succesfully Pollinated Flower "+str(self.flowers_order[i]))
            else :
                rospy.logerr("Failed to Pollinate Flower "+str(self.flowers_order[i]))
                self.flowers_not_pollinated_due_error.append(self.flowers_order[i])
            i += 1
        self.end()

#     def individual_flower_pollination_procedure(self):
#         rospy.loginfo("Pollinating Flower: %f",self.flowers_order[self.number_flowers_pollinated])
#         for _flower in self.flowers.map:
#             if _flower.id ==  self.flowers_order[self.number_flowers_pollinated]:
#                 rospy.loginfo(_flower.point)

#         _control_client = actionlib.SimpleActionClient('ee_go_to_pose', EEGoToPoseAction)
#         print(self.flowers_with_offset[self.number_flowers_pollinated])
#         self.current_flower = self.flowers_with_offset[self.number_flowers_pollinated]

#         #self.current_flower.orientation.x = -1.96422831761e-05
#         #self.current_flower.orientation.y = 0.707111340811
#         #self.current_flower.orientation.z = -0.707102221251
#         #self.current_flower.orientation.w =  3.5750321417e-06


#         # rospy.wait_for_service('find_previsit_poses')
#         # _find_previsit_poses=rospy.ServiceProxy('find_previsit_poses', FindPrevisitPoses)
#         # try:
#         #     _previsit_poses = _find_previsit_poses(self.current_flower)
#         # except rospy.ServiceException as exc:
#         #     print("Service did not process request: " + str(exc))
#         # rospy.loginfo(_previsit_poses.ee_poses)

#         #Move to the offset pose to better estimate flower position
#         for _goal in _previsit_poses.ee_poses:
#             _i=1
#             _control_client.wait_for_server()
#             _send_goal = PoseStamped()
#             _send_goal.pose = _goal
#             # rospy.loginfo(_send_goal)
#             _control_goal = EEGoToPoseGoal(goal_pose=_send_goal)
#             _control_client.send_goal(_control_goal)
#             _control_client.wait_for_result()
#             _control_result=_control_client.get_result()
#             # if (_i==1 or _i==5)and _control_result.goal_reached==False:
#             if _control_result.goal_reached==False:
#                 rospy.logerr("Skipping flower")
#                 self.flowers_not_pollinated_due_error.append(self.flowers_order[self.number_flowers_pollinated])
# #                self.flowers_order = self.flowers_order.append(self.flowers_order[self.number_flowers_pollinated])
# #                self.flowers_with_offset = self.flowers_with_offset.append(self.flowers_with_offset[self.number_flowers_pollinated])
#                 self.number_flowers_pollinated = self.number_flowers_pollinated +1
#                 if self.number_flowers_pollinated < len(self.flowers_order):
#                    rospy.loginfo(self.number_flowers_pollinated)
#                    self.individual_flower_pollination_procedure()
#                 else:
#                    #self.end()
#             self.search_for_flowers()
#             _i = _i+1
#             # rospy.loginfo(_control_result)
#             # rospy.loginfo("current Flower:")
#             # rospy.loginfo(self.current_flower)
#             #rospy.loginfo("Number of Flowers pollinated: %d", self.number_flowers_pollinated)
#             #rospy.loginfo("Total Number of Flowers: %d", len(self.flowers_order))

#         for _flower in self.flowers.map:
#             if _flower.id ==  self.flowers_order[self.number_flowers_pollinated]:
#                 self.current_flower = _flower.pose


#         rospy.loginfo("Final Previsit Pose")
#         rospy.wait_for_service('final_previsit_pose')
#         _final_previsit_pose=rospy.ServiceProxy('final_previsit_pose', FinalPrevisitPose)
#         try:
#             _final_pose = _final_previsit_pose(self.current_flower)
#         except rospy.ServiceException as exc:
#             print("Service did not process request: " + str(exc))
#         # rospy.loginfo(_final_pose.ee_previsit_pose)

#         #Go To position to start pollinating
#         _send_goal = PoseStamped()
#         _send_goal.pose = _final_pose.ee_previsit_pose
#         _control_client.wait_for_server()
#         _control_goal = EEGoToPoseGoal(goal_pose=_send_goal)
#         _control_client.send_goal(_control_goal)
#         _control_client.wait_for_result()
#         _control_result=_control_client.get_result()
#         # rospy.loginfo(_control_result)

#         if _control_result.goal_reached==True:
#             self.visual_servoing()
#         else:
#             self.flowers_not_pollinated_due_error.append(self.flowers_order[self.number_flowers_pollinated])
#             self.number_flowers_pollinated = self.number_flowers_pollinated +1
#             if self.number_flowers_pollinated < len(self.flowers_order):
#                rospy.loginfo(self.number_flowers_pollinated)
#                self.individual_flower_pollination_procedure()
#             else:
#                self.end()


    def search_for_flowers(self):
        rospy.loginfo("Call search_for_flowers Service")
        rospy.wait_for_service('searchFF')
        start_search_for_flowers=rospy.ServiceProxy('searchFF', SearchForFlowers)
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
        self.flowers_with_offset = []
        #self.home_position()
        self.start()

    def home_position(self):
        rospy.loginfo("Go to Home Position/Unwind")
        _pose_goal = Pose()
        _pose_goal.position.x = 0.497
        _pose_goal.position.y = 0.029
        _pose_goal.position.z = 0.550
        _pose_goal.orientation.x = -0.094
        _pose_goal.orientation.y = 0.860
        _pose_goal.orientation.z = -0.012
        _pose_goal.orientation.w = 0.501
        self.send_pose(_pose_goal)

        # _client = actionlib.SimpleActionClient( "/j2n6s300_driver/joints_action/joint_angles",kinova_msgs.msg.ArmJointAnglesAction)
        # _client.wait_for_server()
        # _goal = kinova_msgs.msg.ArmJointAnglesGoal()
        # _goal.angles.joint1 =  270.0
        # _goal.angles.joint2 =  150.0
        # _goal.angles.joint3 =  60.0
        # _goal.angles.joint4 =  120.0
        # _goal.angles.joint5 =  0.0
        # _goal.angles.joint6 =  -90.0
        # _goal.angles.joint7 =  0
        # _client.send_goal(_goal)
        # if _client.wait_for_result(rospy.Duration(20.0)):
        #     _result = _client.get_result()
        #     return
        # else:
        #     rospy.logerr('the joint angle action timed-out')
        #     _client.cancel_all_goals()
        #     return


    def shutdown(self):
        rospy.loginfo("Pollination State Machine node is shutdown")
        rospy.sleep(1)

def main():
    try:
        pollination_manipulation = Pollination_manipulation()

    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
