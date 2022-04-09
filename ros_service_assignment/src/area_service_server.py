#!/usr/bin/env python

from ros_service_assignment.srv import RectangleAreaService
from ros_service_assignment.srv import RectangleAreaServiceRequest
from ros_service_assignment.srv import RectangleAreaServiceResponse

import rospy


#callback function similar to subscriber
def handle_calc_area(req):
    print ("Returning [%s x %s = %s]"%(req.width, req.height, (req.width * req.height)))
    return RectangleAreaServiceResponse(req.width * req.height)

def area_service_server():
    rospy.init_node('area_service_server')
    s = rospy.Service('rectangle_area_service', RectangleAreaService, handle_calc_area)  #name of srv, srv type, srv callback func
    print ("Ready to calc area")
    rospy.spin()
    
if __name__ == "__main__":
    area_service_server()