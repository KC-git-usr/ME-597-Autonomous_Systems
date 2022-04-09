#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState

def position_node():
        pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=1)
        rospy.init_node('model_service_node', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
                state_msg = ModelState()
                state_msg.model_name = 'turtlebot3_burger'
                rospy.wait_for_service('/gazebo/set_model_state')
                state_msg.pose.position.x = 1
                state_msg.pose.position.y = 1
                try:
                        set_state = rospy.ServiceProxy('/gazebo/set_model_state',
                        SetModelState)
                        resp = set_state(state_msg)
                except rospy.ServiceException:
                        print("Service call failed: ")
                        rate.sleep()

if __name__ == '__main__':
        try:
                position_node()
        except rospy.ROSInterruptException:
                pass
