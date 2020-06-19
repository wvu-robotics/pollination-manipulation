#!/usr/bin/env python

import rospy
from manipulation_mapping.msg import BuildMapAction, BuildMapGoal
from manipulation_planning.msg import PlanFlowerSequenceAction, PlanFlowerSequenceGoal
from manipulation_control.msg import EEGoToPoseAction, EEGoToPoseGoal
from manipulation_vision.msg import ApproachFlowerAction, ApproachFlowerGoal
from manipulation_pollinator.srv import PollinateFlower
from manipulation_vision.srv import SearchForFlowers
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
from manipulation_mapping.msg import FlowerMap
from manipulation_mapping.msg import Flower


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
            self.start_pollination = rospy.wait_for_message("start_pollination_procedures",Bool)
            if self.start_pollination.data == True:
                self.planning()


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
        self.end()

    def end(self):
        rospy.loginfo("Pollination Procedures ended")




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
