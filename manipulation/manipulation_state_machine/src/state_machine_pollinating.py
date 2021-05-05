#!/usr/bin/env python

import rospy
from manipulation_mapping.msg import BuildMapAction, BuildMapGoal
from manipulation_common.msg import PlanFlowerSequenceAction, PlanFlowerSequenceGoal
from manipulation_control.msg import EEGoToPoseAction, EEGoToPoseGoal
from manipulation_vision.msg import ApproachFlowerAction, ApproachFlowerGoal
from manipulation_pollinator.srv import PollinateFlower
from manipulation_common.srv import SearchForFlowers
import kinova_msgs.msg



from std_msgs.msg import Bool
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3Stamped
from manipulation_common.msg import FlowerMap
from manipulation_common.msg import Flower

from visualization_msgs.msg import Marker


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
        #set the robot to home position by Joint Control
        self.start()

    def global_flower_poses(self,data):
        self.flowers = data

    def start(self):
        while not rospy.is_shutdown():
            #self.home_position()
            self.start_pollination = rospy.wait_for_message("start_pollination_procedures",Bool)
            self.end_pollination.data = False
            self.pollination_end_pub.publish(self.end_pollination)
            if self.start_pollination.data == True:
                self.mapping()
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

    def individual_flower_pollination_procedure(self):
        rospy.loginfo("Pollinating Flower: %f",self.flowers_order[self.number_flowers_pollinated])
        flower_vector = Vector3Stamped() # define a vector for visual servoing offset
        for _flower in self.flowers.map:
            if _flower.id ==  self.flowers_order[self.number_flowers_pollinated]:
                rospy.loginfo(_flower.point)
                flower_vector = _flower.vec
        _control_client = actionlib.SimpleActionClient('ee_go_to_pose', EEGoToPoseAction)
        print(self.flowers_with_offset[self.number_flowers_pollinated])
        self.current_flower = self.flowers_with_offset[self.number_flowers_pollinated]


        #Visualization tools:
        marker_pub = rospy.Publisher("marker_flower_state_machine", Marker)
        marker = Marker()
        marker.header.frame_id = "j2n6s300_link_base"
        marker.type = 2
        marker.action = 0
        color = ColorRGBA()
        color.r = 1.0
        color.g = 0.0
        color.b = 0.0
        color.a = 1.0
        marker.color = color
        marker.pose.position = self.current_flower.position
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker_pub.publish(marker)

        _send_goal = PoseStamped()
        _send_goal.pose = self.current_flower
        _control_client.wait_for_server()
        _control_goal = EEGoToPoseGoal(goal_pose=_send_goal)
        _control_client.send_goal(_control_goal)
        _control_client.wait_for_result()
        _control_result=_control_client.get_result()
        self.search_for_flowers()
        if _control_result.goal_reached==True:
            self.approach(flower_vector)
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
        rospy.wait_for_service('searchFF')
        start_search_for_flowers=rospy.ServiceProxy('searchFF', SearchForFlowers)
        try:
            search_for_flowers_status = start_search_for_flowers()
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        #Add some exception
        return

    def approach(self,flowervector):
        rospy.loginfo("Approach Flower: %d",self.flowers_order[self.number_flowers_pollinated])
        _approach_client = actionlib.SimpleActionClient('approach_flower', ApproachFlowerAction)
        _approach_client.wait_for_server()
        print(self.flowers_order[self.number_flowers_pollinated])
        _approach_goal = ApproachFlowerGoal(flower_pose=self.current_flower,flower_vector = flowervector)
        _approach_client.send_goal(_approach_goal)
        rospy.sleep(1)
        _approach_client.wait_for_result()
        _approach_result=_approach_client.get_result().flower_approached
        rospy.loginfo("Visual servoing result: %r", _approach_result)
        if _approach_result==True:
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
        self.home_position()
        self.start()

    def home_position(self):
        rospy.loginfo("Go to Home Position/Unwind")
        _client = actionlib.SimpleActionClient( "/j2n6s300_driver/joints_action/joint_angles",kinova_msgs.msg.ArmJointAnglesAction)
        _client.wait_for_server()
        _goal = kinova_msgs.msg.ArmJointAnglesGoal()
        _goal.angles.joint1 =  270.0
        _goal.angles.joint2 =  150.0
        _goal.angles.joint3 =  60.0
        _goal.angles.joint4 =  120.0
        _goal.angles.joint5 =  0.0
        _goal.angles.joint6 =  -90.0
        _goal.angles.joint7 =  0.0
        _client.send_goal(_goal)
        if _client.wait_for_result(rospy.Duration(20.0)):
            _result = _client.get_result()
            return
        else:
            rospy.logerr('the joint angle action timed-out')
            _client.cancel_all_goals()
            return


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
