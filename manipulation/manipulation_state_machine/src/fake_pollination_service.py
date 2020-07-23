#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from manipulation_services.srv import pollinating, pollinatingResponse

class Run_Pollination_Sevice:
    def __init__(self):
        rospy.init_node('Pollination_Server',anonymous=True)
        rospy.loginfo("Fake_Pollinating_service_initialized")
        s = rospy.Service('pollinating', pollinating, self.pollinating_handle)
        rospy.spin()

    def pollinating_handle(self,req):
        rospy.loginfo("Start_pollination")
        response = pollinatingResponse()
        rospy.sleep(3.0)
        response.finished = True
        rospy.loginfo("pollination Finished")
        return response

def main():
    try:
        run_pollination_sevice = Run_Pollination_Sevice()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
