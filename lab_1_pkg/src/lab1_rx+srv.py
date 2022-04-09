#!/usr/bin/env python3
import rospy
from lab_1_pkg.msg import CustomMessage
from lab_1_pkg.srv import CustomService, CustomServiceResponse

def my_rx_callback(data):
    rospy.loginfo('Received ctr:{}, and text:{}'.format(data.ctr,data.text))

def my_rx_server_callback(req):
    rospy.loginfo('service request received arg_1:{}'.format(req.arg_1))
    return CustomServiceResponse('service executed')

def my_rx_node():
    rospy.init_node('my_rx_node', anonymous=True)
    rospy.Subscriber("tx_msg", CustomMessage, my_rx_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        my_rx_node()
    except rospy.ROSInterruptException:
        pass