#!/usr/bin/env python

import sys
import rospy
from ros_service_assignment.srv import RectangleAreaService
from ros_service_assignment.srv import RectangleAreaServiceRequest
from ros_service_assignment.srv import RectangleAreaServiceResponse

def area_service_client(x, y):
    rospy.wait_for_service('rectangle_area_service')
    try:
        rectangle_area_service = rospy.ServiceProxy('rectangle_area_service', RectangleAreaService) #establishes the connection to the server
        resp1 = rectangle_area_service(x, y)
        return resp1.area
    except rospy.ServiceException:
        print ("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
    else:
        print (usage())
        sys.exit(1)
    print ("Requesting %sx%s"%(x, y))
    s = area_service_client(x, y)
    print ("%s + %s = %s"%(x, y, s))