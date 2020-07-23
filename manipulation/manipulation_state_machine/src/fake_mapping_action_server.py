#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from manipulation_services.msg import mappingAction, mappingResult
from std_msgs.msg import Int32

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import random
import numpy as np



import actionlib

class Run_Map_Service:
    def __init__(self):
        rospy.init_node('map_server',anonymous=True)
        rospy.loginfo("Fake_Mapping_Node_initialized")
        self.server = actionlib.SimpleActionServer('mapping', mappingAction, self.mapping_handle, False)
        self.result = mappingResult()
        self.pose_array_publisher = rospy.Publisher("/bramblebee/arm/global_flower_poses", PoseArray, queue_size=1)
        random.seed(10)
        self.server.start()


        rospy.spin()

    def mapping_handle(self,goal):
        rospy.loginfo("Data Received1")
        rospy.loginfo(goal.n_points)
        rospy.sleep(5.0)
        if goal.n_points>=100000:
            rospy.loginfo("Map Constructed")
            self.result.mapping_completed = True
            self.publish_some_pose_to_a_node()
        self.server.set_succeeded(self.result)

    def publish_some_pose_to_a_node(self):
        _pose_array = PoseArray()

        _pose_array.header.frame_id = "/base_link"
        _pose_array.header.stamp = rospy.Time.now()

        _pose1 = Pose()
        _pose1.position.x = random.random()
        _pose1.position.y = random.random()
        _pose1.position.z = random.random()
        _pose1.orientation.x = random.random()
        _pose1.orientation.y = random.random()
        _pose1.orientation.z = random.random()
        _pose1.orientation.w = random.random()
        _pose2 = Pose()
        _pose2.position.x = random.random()
        _pose2.position.y = random.random()
        _pose2.position.z = random.random()
        _pose2.orientation.x = random.random()
        _pose2.orientation.y = random.random()
        _pose2.orientation.z = random.random()
        _pose2.orientation.w = random.random()
        _pose3 = Pose()
        _pose3.position.x = random.random()
        _pose3.position.y = random.random()
        _pose3.position.z = random.random()
        _pose3.orientation.x = random.random()
        _pose3.orientation.y = random.random()
        _pose3.orientation.z = random.random()
        _pose3.orientation.w = random.random()

        _pose_array.poses.append(_pose1)
        _pose_array.poses.append(_pose2)
        _pose_array.poses.append(_pose3)

        self.pose_array_publisher.publish(_pose_array)

def main():
    try:
        run_map_service = Run_Map_Service()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
