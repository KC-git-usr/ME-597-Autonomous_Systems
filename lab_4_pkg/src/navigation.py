#!/usr/bin/env python3

import sys
import os
import numpy as np

import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist

class Navigation:
    """! Navigation node class.
    This class should serve as a template to implement the path planning and 
    path follower components to move the turtlebot from position A to B.
    """
    def __init__(self, node_name='Navigation'):
        """! Class constructor.
        @param  None.
        @return An instance of the Navigation class.
        """
        # ROS related variables
        self.node_name = node_name
        self.rate = 0
        # Path planner/follower related variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()


    def init_app(self):
        """! Node intialization.
        @param  None
        @return None.
        """
        # ROS node initilization
        
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)
        # Subscribers
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_pose_cbk, queue_size=1)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__ttbot_pose_cbk, queue_size=1)
        # Publishers
        self.path_pub = rospy.Publisher('global_plan', Path, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def __goal_pose_cbk(self, data):
        """! Callback to catch the goal pose.
        @param  data    PoseStamped object from RVIZ.
        @return None.
        """
        self.goal_pose = data
        rospy.loginfo('goal_pose:{:.4f},{:.4f}'.format(self.goal_pose.pose.position.x,self.goal_pose.pose.position.y))

    def __ttbot_pose_cbk(self, data):
        """! Callback to catch the position of the vehicle(turtlebot).
        @param  data    PoseWithCovarianceStamped object from amcl.
        @return None.
        """
        # TODO: MAKE SURE YOUR POSITION ESTIMATE IS GOOD ENOUGH.
        self.ttbot_pose = data.pose
        cov = data.pose.covariance
        rospy.loginfo('ttbot_pose:{:.4f},{:.4f}'.format(self.ttbot_pose.pose.position.x,self.ttbot_pose.pose.position.y))
        rospy.loginfo('ttbot_pose:{}'.format(cov))
    
    def a_star_path_planner(self,start_pose,end_pose):
        """! A Start path planner.
        @param  start_pose    PoseStamped object containing the start of the path to be created.
        @param  end_pose      PoseStamped object containing the end of the path to be created.
        @return path          Path object containing the sequence of waypoints of the created path.
        """
        path = Path()
        rospy.loginfo('A* planner.\n> start:{},\n> end:{}'.format(start_pose.pose.position,end_pose.pose.position))
        # TODO: IMPLEMENTATION OF THE ASTAR ALGORITHM
        path.poses.append(start_pose)
        path.poses.append(end_pose)
        return path
    
    def get_path_idx(self,path,vehicle_pose):
        """! Path follower.
        @param  path                  Path object containing the sequence of waypoints of the created path.
        @param  current_goal_pose     PoseStamped object containing the current vehicle position.
        @return idx                   Position int the path pointing to the next goal pose to follow.
        """
        idx = 0
        # TODO: IMPLEMENT A MECHANISM TO DECIDE WHICH POINT IN THE PATH TO FOLLOW idx<=len(path)
        return idx

    def path_follower(self,vehicle_pose, current_goal_pose):
        """! Path follower.
        @param  vehicle_pose           PoseStamped object containing the current vehicle pose.
        @param  current_goal_pose      PoseStamped object containing the current target from the created path. This is different from the global target.
        @return path                   Path object containing the sequence of waypoints of the created path.
        """
        speed = 0.3
        heading = 0
        # TODO: IMPLEMENT PATH FOLLOWER
        return speed,heading

    def move_ttbot(self,speed,heading):
        """! Function to move turtlebot passing directly a heading angle and the speed.
        @param  heading     Desired yaw angle.
        @param  speed       Desired speed.
        @return path      object containing the sequence of waypoints of the created path.
        """
        cmd_vel = Twist()
        # TODO: IMPLEMENT YOUR CONTROLLER LOW LEVEL CONTROLLER
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0

        self.cmd_vel_pub.publish(cmd_vel)


    def run(self):
        """! Main loop of the node. You need to wait until a new pose is published, create a path and then
        drive the vehicle towards the final pose.
        @param none
        @return none
        """
        
        '''
            Main loop
        '''
        path_complete = False
        timeout = False
        idx = 0
        while not rospy.is_shutdown():
            # 1. Create the path to follow
            path = self.a_star_path_planner(self.ttbot_pose,self.goal_pose)
            # 2. Loop through the path and move the robot
            idx = self.get_path_idx(path,self.ttbot_pose)
            current_goal = path.poses[idx]
            speed,heading = self.path_follower(self.ttbot_pose,current_goal)
            self.move_ttbot(speed,heading)
            # ----------------------------------------
            # TODO: YOU NEED TO ADD YOUR SOLUTION HERE
            # ----------------------------------------
            self.rate.sleep() 
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.name))


if __name__ == "__main__":
    nav = Navigation(node_name='Navigation')
    nav.init_app()
    try:
        nav.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)