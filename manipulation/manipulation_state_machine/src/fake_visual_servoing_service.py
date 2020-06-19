#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from manipulation_services.srv import visual_servoing, visual_servoingResponse

class Run_Visual_Servoing_Sevice:
    def __init__(self):
        rospy.init_node('Visual_servoing',anonymous=True)
        rospy.loginfo("Fake_Visual_Servoing_Node_initialized")
        s = rospy.Service('visual_servoing', visual_servoing, self.visual_servoing_handle)
        rospy.spin()

    def visual_servoing_handle(self,req):
        rospy.loginfo("Start_visual_servoing")
        response = visual_servoingResponse()
        response.finished = True
        rospy.sleep(3.0)
        rospy.loginfo("visual_servoing_Completed")
        return response

def main():
    try:
        run_visual_servoing_sevice = Run_Visual_Servoing_Sevice()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
