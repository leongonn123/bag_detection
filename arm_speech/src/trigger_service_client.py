#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger

def trigger_service():
    rospy.init_node('trigger_service_client')
    rospy.wait_for_service('control_and_speak')
    try:
        control_and_speak = rospy.ServiceProxy('control_and_speak', Trigger)
        response = control_and_speak()
        print("Service response:", response.success, response.message)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == "__main__":
    trigger_service()

