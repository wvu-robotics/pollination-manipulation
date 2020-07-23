#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from manipulation_services.srv import SearchForFlowers, SearchForFlowersResponse

class Run_Search_For_Flowers_Service:
    def __init__(self):
        rospy.init_node('fake_search_for_flowers_service',anonymous=True)
        rospy.loginfo("fake_search_for_flowers_service node initialized")
        s = rospy.Service('SearchForFlowers', SearchForFlowers, self.SearchForFlowers_handle)
        rospy.spin()

    def SearchForFlowers_handle(self,req):
        rospy.loginfo("Start_Search_For_Flowers")
        response = SearchForFlowersResponse()
        rospy.sleep(1.0)
        response.success = True
        rospy.loginfo("Start_Search_For_Flowers Finished")
        return response

def main():
    try:
        run_search_for_flowers = Run_Search_For_Flowers_Service()
    except rospy.ROSInterruptException:
        pass
if __name__ == '__main__':
    main()
