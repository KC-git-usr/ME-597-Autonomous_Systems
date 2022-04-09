#!/usr/bin/env python3
import rospy
from lab_1_pkg.msg import CustomMessage
from lab_1_pkg.srv import CustomService, CustomServiceRequest, CustomServiceResponse
from std_msgs.msg import String

def call_custom_service(arg_srv):
    rospy.wait_for_service('custom_service')
    try:
        cust_srv = rospy.ServiceProxy('custom_service', CustomService)
        srv_resp = cust_srv(arg_srv)
        return srv_resp.out_1
    except rospy.ServiceException as e:
        pass

def my_tx_node():
    pub = rospy.Publisher('tx_msg', CustomMessage, queue_size=10)
    rospy.init_node('my_tx_node', anonymous=True)
    rate = rospy.Rate(10)
    ctr = 0
    while not rospy.is_shutdown():
        msg = CustomMessage()
        msg.ctr = ctr
        ctr = ctr + 1
        msg.text = 'Message from node my_tx_node'
        rospy.loginfo('[my_tx_node] Running')
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        my_tx_node()
    except rospy.ROSInterruptException:
        pass

