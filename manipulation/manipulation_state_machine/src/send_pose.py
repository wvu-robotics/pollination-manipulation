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
    	rospy.init_node('send_pose', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.current_flower = Pose()

        self.start()


    def start(self):
        while not rospy.is_shutdown():
            self.send_goal()


    def send_goal(self):
        _control_client = actionlib.SimpleActionClient('ee_go_to_pose', EEGoToPoseAction)
        _send_goal = PoseStamped()
        _send_goal.header.frame_id = "j2n6s300_link_base"
        _send_goal.pose.position.x = -0.133804635662
        _send_goal.pose.position.y = -0.541619423198
        _send_goal.pose.position.z =0.180772987331




#        _send_goal.pose.position.x = -0.214588553227 
#        _send_goal.pose.position.y = -0.505679755382
#        _send_goal.pose.position.z =  0.52512740288
        _send_goal.pose.orientation.x = -0.5
        _send_goal.pose.orientation.y = -0.5
        _send_goal.pose.orientation.z = 0.5
        _send_goal.pose.orientation.w = -0.5 

        #Visualization tools:
        marker_pub = rospy.Publisher("marker_flower_state_machine", Marker)
        marker = Marker()
        marker.header.frame_id = "root"
        marker.type = 2
        marker.action = 0
        color = ColorRGBA()
        color.r = 1.0
        color.g = 0.0
        color.b = 0.0
        color.a = 1.0
        marker.color = color
        marker.pose.position = _send_goal.pose.position
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        
        marker_pub.publish(marker)

        print(marker)
        print("Goal: {}".format(_send_goal))
        _control_client.wait_for_server()
        _control_goal = EEGoToPoseGoal(goal_pose=_send_goal)
        _control_client.send_goal(_control_goal)
        _control_client.wait_for_result()
        _control_result=_control_client.get_result()
        if _control_result.goal_reached==True:
            print("goal complete")
       

    def visual_servoing(self,flowervector):
        rospy.loginfo("Visual_servoing")
        rospy.loginfo("Visual Servoing Flower: %d",self.flowers_order[self.number_flowers_pollinated])

        _visual_servoing_client = actionlib.SimpleActionClient('approach_flower', ApproachFlowerAction)
        _visual_servoing_client.wait_for_server()
        print(self.flowers_order[self.number_flowers_pollinated])
        _visual_servoing_goal = ApproachFlowerGoal(flower_pose=self.current_flower,flower_vector = flowervector)
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
