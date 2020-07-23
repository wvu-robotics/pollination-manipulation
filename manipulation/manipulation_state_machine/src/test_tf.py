#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from tf import transformations as t
import numpy as np
import math



class TF_test():
    def __init__(self):

        rospy.init_node("TF_test",anonymous=True)
        self.flower = PoseStamped().pose
        self.flower.orientation.w=1

        self.pose1 = PoseStamped().pose
        _position = np.matrix(t.translation_matrix([self.flower.position.x,self.flower.position.y,self.flower.position.z]))
        rospy.loginfo(_position)
        _translation = np.matrix(t.translation_matrix([0,-0.2,0]))
        rospy.loginfo(_translation)
        _rotation_matrix = np.matrix(t.euler_matrix(math.radians(-90),0,0))
        rospy.loginfo(_rotation_matrix)
        _quaternion_matrix = np.matrix(t.quaternion_matrix([self.flower.orientation.x,        self.flower.orientation.y,
        self.flower.orientation.z,        self.flower.orientation.w]))
        rospy.loginfo(_quaternion_matrix)
        _final1 = _quaternion_matrix*_position*_translation*_rotation_matrix
        rospy.loginfo(_final1)
        rospy.loginfo(t.translation_from_matrix(_final1))
        rospy.loginfo(t.quaternion_from_matrix(_final1))
        self.pose1.position.x = t.translation_from_matrix(_final1)[0]
        self.pose1.position.y = t.translation_from_matrix(_final1)[1]
        self.pose1.position.z = t.translation_from_matrix(_final1)[2]
        self.pose1.orientation.x = t.quaternion_from_matrix(_final1)[0]
        self.pose1.orientation.y = t.quaternion_from_matrix(_final1)[1]
        self.pose1.orientation.z = t.quaternion_from_matrix(_final1)[2]
        self.pose1.orientation.w = t.quaternion_from_matrix(_final1)[3]



        self.pose2 = PoseStamped().pose
        _position = np.matrix(t.translation_matrix([self.flower.position.x,self.flower.position.y,self.flower.position.z]))
        rospy.loginfo(_position)
        _translation = np.matrix(t.translation_matrix([-0.2,-0.2,-0.2]))
        rospy.loginfo(_translation)
        _rotation_matrix = np.matrix(t.euler_matrix(math.radians(-60),0,math.radians(-30)))
        rospy.loginfo(_rotation_matrix)
        _quaternion_matrix = np.matrix(t.quaternion_matrix([self.flower.orientation.x,        self.flower.orientation.y,
        self.flower.orientation.z,        self.flower.orientation.w]))
        rospy.loginfo(_quaternion_matrix)
        _final2 = _quaternion_matrix*_position*_translation*_rotation_matrix
        rospy.loginfo(_final2)
        rospy.loginfo(t.translation_from_matrix(_final2))
        rospy.loginfo(t.quaternion_from_matrix(_final2))
        self.pose2.position.x = t.translation_from_matrix(_final2)[0]
        self.pose2.position.y = t.translation_from_matrix(_final2)[1]
        self.pose2.position.z = t.translation_from_matrix(_final2)[2]
        self.pose2.orientation.x = t.quaternion_from_matrix(_final2)[0]
        self.pose2.orientation.y = t.quaternion_from_matrix(_final2)[1]
        self.pose2.orientation.z = t.quaternion_from_matrix(_final2)[2]
        self.pose2.orientation.w = t.quaternion_from_matrix(_final2)[3]

        self.pose3 = PoseStamped().pose
        _position = np.matrix(t.translation_matrix([self.flower.position.x,self.flower.position.y,self.flower.position.z]))
        rospy.loginfo(_position)
        _translation = np.matrix(t.translation_matrix([0.2,-0.2,-0.2]))
        rospy.loginfo(_translation)
        _rotation_matrix = np.matrix(t.euler_matrix(math.radians(-60),0,math.radians(30)))
        rospy.loginfo(_rotation_matrix)
        _quaternion_matrix = np.matrix(t.quaternion_matrix([self.flower.orientation.x,        self.flower.orientation.y,
        self.flower.orientation.z,        self.flower.orientation.w]))
        rospy.loginfo(_quaternion_matrix)
        _final2 = _quaternion_matrix*_position*_translation*_rotation_matrix
        rospy.loginfo(_final2)
        rospy.loginfo(t.translation_from_matrix(_final2))
        rospy.loginfo(t.quaternion_from_matrix(_final2))
        self.pose3.position.x = t.translation_from_matrix(_final2)[0]
        self.pose3.position.y = t.translation_from_matrix(_final2)[1]
        self.pose3.position.z = t.translation_from_matrix(_final2)[2]
        self.pose3.orientation.x = t.quaternion_from_matrix(_final2)[0]
        self.pose3.orientation.y = t.quaternion_from_matrix(_final2)[1]
        self.pose3.orientation.z = t.quaternion_from_matrix(_final2)[2]
        self.pose3.orientation.w = t.quaternion_from_matrix(_final2)[3]

        self.pose4 = PoseStamped().pose
        _position = np.matrix(t.translation_matrix([self.flower.position.x,self.flower.position.y,self.flower.position.z]))
        rospy.loginfo(_position)
        _translation = np.matrix(t.translation_matrix([0,-0.2,0.2]))
        rospy.loginfo(_translation)
        _rotation_matrix = np.matrix(t.euler_matrix(math.radians(-120),0,0))
        rospy.loginfo(_rotation_matrix)
        _quaternion_matrix = np.matrix(t.quaternion_matrix([self.flower.orientation.x,        self.flower.orientation.y,
        self.flower.orientation.z,        self.flower.orientation.w]))
        rospy.loginfo(_quaternion_matrix)
        _final2 = _quaternion_matrix*_position*_translation*_rotation_matrix
        rospy.loginfo(_final2)
        rospy.loginfo(t.translation_from_matrix(_final2))
        rospy.loginfo(t.quaternion_from_matrix(_final2))
        self.pose4.position.x = t.translation_from_matrix(_final2)[0]
        self.pose4.position.y = t.translation_from_matrix(_final2)[1]
        self.pose4.position.z = t.translation_from_matrix(_final2)[2]
        self.pose4.orientation.x = t.quaternion_from_matrix(_final2)[0]
        self.pose4.orientation.y = t.quaternion_from_matrix(_final2)[1]
        self.pose4.orientation.z = t.quaternion_from_matrix(_final2)[2]
        self.pose4.orientation.w = t.quaternion_from_matrix(_final2)[3]



        rospy.loginfo(self.flower)



        self.br = tf.TransformBroadcaster()
        rospy.loginfo("TF publisher node is on")
        self.r = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown)
        self.publish()



    def publish(self):
        while not rospy.is_shutdown():
            self.static_frames()
            self.r.sleep()
        #    self.dynamic_frames()

    def static_frames(self):
            rospy.loginfo(self.pose1)
            self.br.sendTransform((self.flower.position.x,self.flower.position.y,self.flower.position.z),
                                   (self.flower.orientation.x,
                                   self.flower.orientation.y,
                                   self.flower.orientation.z,
                                   self.flower.orientation.w),rospy.Time.now(),"Flower","world")
            self.br.sendTransform((self.pose1.position.x,self.pose1.position.y,self.pose1.position.z),
                                (self.pose1.orientation.x,
                                self.pose1.orientation.y,
                                self.pose1.orientation.z,
                                self.pose1.orientation.w),rospy.Time.now(),"Pose1","world")
            self.br.sendTransform((self.pose2.position.x,self.pose2.position.y,self.pose2.position.z),
                                (self.pose2.orientation.x,
                                self.pose2.orientation.y,
                                self.pose2.orientation.z,
                                self.pose2.orientation.w),rospy.Time.now(),"Pose2","world")
            self.br.sendTransform((self.pose3.position.x,self.pose3.position.y,self.pose3.position.z),
                                (self.pose3.orientation.x,
                                self.pose3.orientation.y,
                                self.pose3.orientation.z,
                                self.pose3.orientation.w),rospy.Time.now(),"pose3","world")
            self.br.sendTransform((self.pose4.position.x,self.pose4.position.y,self.pose4.position.z),
                                (self.pose4.orientation.x,
                                self.pose4.orientation.y,
                                self.pose4.orientation.z,
                                self.pose4.orientation.w),rospy.Time.now(),"pose4","world")


    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("TF Test Publisher Node is shutdown")
        rospy.sleep(1)

def main():
    try:
        TF_static_Publisher = TF_test()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
