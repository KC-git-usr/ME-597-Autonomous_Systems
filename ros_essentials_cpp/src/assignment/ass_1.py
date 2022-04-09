#!/usr/bin/env python

#requirements
#publisher- that will make the robot move
#subscriber- that will show the location of the robot

import rospy
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def listener_callback(message):
    rospy.loginfo("Location is : ")
    print(message.x, message.y, message.theta)


def remote():
    speed_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, listener_callback)

    rospy.init_node('remote_node', anonymous=True)

    rate = rospy.Rate(1) # 1hz

    twist = Twist()

    while not rospy.is_shutdown():
        twist.linear.x = random.randint(-5,5)
        twist.linear.y = random.randint(-5,5)
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = random.randint(-5,5)
        speed_pub.publish(twist)
        print("Node running")
        rate.sleep()

if __name__ == '__main__':
    try:
        remote()
    except rospy.ROSInterruptException:
        pass
