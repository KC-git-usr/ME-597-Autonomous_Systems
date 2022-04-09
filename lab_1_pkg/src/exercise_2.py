#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState      #importing services from specified location
from gazebo_msgs.msg import ModelState     #importing msg type from specified location


'''
This node does the following: 
1. publishes to /set_model_state topic via ModelState msg
2. initializes a client that send service requests through:
        a. SpwanModel srv
        b. DeleteModel srv
        c. SetModelState srv
'''
def position_node():

        # Create a publisher object with ModelState
        pub_model = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size=10)  #topic name, msg type, buffer size

        # Declare the node, and register it with a unique name
        rospy.init_node('model_service_node', anonymous=True)

        rate = rospy.Rate(5)  #executing at a rate of 5Hz

        state_msg = ModelState()  #creating object

        #specifing parameters of object
        state_msg.model_name = 'turtlebot3_burger'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0

        while not rospy.is_shutdown():

                rospy.wait_for_service('/gazebo/set_model_state')   #wait until a service becomes available
                
                if (state_msg.pose.position.x<=1):   #checking if we've reached target position yet
                        try:
                                set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)   #create a callable proxy to a service
                                resp = set_state(state_msg)  #setting the state of the bot
                        except rospy.ServiceException:
                                print("Service call failed: ")

                else:   #stopping the node once we've reached target position
                        break

                state_msg.pose.position.x += 0.05   #updating position of bot
                rate.sleep()


if __name__ == '__main__':
        try:
                position_node()
        except rospy.ROSInterruptException:
                pass
