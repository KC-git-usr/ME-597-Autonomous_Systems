#!/usr/bin/env python3

import sys
import math
import random
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist

from Controller import PidController

class Navigation:

    def __init__(self, node_name='Navigation'):
        # ROS related variables
        self.node_name = node_name
        self.rate = 0
        # Path planner/follower related variables
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()


    def init_app(self):

        # ROS node initilization
        
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)
        # Subscribers
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__ttbot_pose_cbk, queue_size=1)
        # Publishers
        self.path_pub = rospy.Publisher('global_plan', Path, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def goal_pose_cbk(self):
        radius = math.sqrt(3)
        random_heading = random.uniform(-2*math.pi, 2*math.pi)
        self.goal_pose.pose.position.x = self.ttbot_pose.pose.position.x + radius*math.cos(random_heading)
        self.goal_pose.pose.position.y = self.ttbot_pose.pose.position.y + radius*math.sin(random_heading)

    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose = data.pose
        cov = data.pose.covariance
    

    def path_follower(self):
        i = 0
        for y,x,heading in transformed_path:
            i = i+1
            while(self.align(heading)):
                pass
            #'''
            while self.move(y, x):
                if i==(len(transformed_path)):
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0
                    self.cmd_vel_pub.publish(cmd_vel)
                    return True
            #'''

    def move_ttbot(self,target_y=0, target_x=0):
        cmd_vel = Twist()
        dt = 1/100
        d = ( (target_y-self.ttbot_pose.pose.position.y)**2 + (target_x-self.ttbot_pose.pose.position.x)**2)**(0.5)
        error = abs(d)
        if (error>0.15):
            PID_obj_2 = PidController(0.1, 0.008, 0.001, dt, 0.0, 20)
            cmd_vel.linear.x = PID_obj_2.step(error)
            print("Moving to : ", target_y, target_x)
            self.cmd_vel_pub.publish(cmd_vel)
            return 1
        else:
            cmd_vel.linear.x = 0
            self.cmd_vel_pub.publish(cmd_vel)
            print("Done moving to position")
            return 0

    def align_ttbot(self, target_heading=0):
        cmd_vel = Twist()
        dt = 1/100
        error = (target_heading-self.current_heading)
        if (abs(error)>5):
            PID_obj_1 = PidController(0.0007, 0.0006, 0.0001, dt, -2, 2)
            cmd_vel.angular.z = PID_obj_1.step(error)
            print("Pls wait, aligning to : ", target_heading)
            self.cmd_vel_pub.publish(cmd_vel)
            return 1
        else:
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)
            print("Done aligning")
            return 0

    def run(self):
        scan_complete = False
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
    nav = Navigation(node_name='Auto Mapping Node')
    nav.init_app()
    try:
        nav.run()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)