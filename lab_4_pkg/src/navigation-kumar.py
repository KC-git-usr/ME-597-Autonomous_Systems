#!/usr/bin/env python3

import re
import sys
import os
import numpy as np

import math
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist

from Controlllers import PidController
from astar_map import trigger_a_star

class Navigation:

    def __init__(self, node_name='Navigation'):
        self.node_name = node_name
        self.rate = 0
        self.path = Path()
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.current_heading = 0
        self.goal_heading = 0
        self.end_pt = '0,0'


    def init_app(self):
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(100)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_pose_cbk, queue_size=50)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__ttbot_pose_cbk, queue_size=50)
        self.path_pub = rospy.Publisher('global_plan', Path, queue_size=50)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=50)


    def __goal_pose_cbk(self, data):
        self.goal_pose = data
        print("Goal pose x: ", self.goal_pose.pose.position.x)
        print("Goal pose y: ", self.goal_pose.pose.position.y)
        q0 = self.goal_pose.pose.orientation.x
        q1 = self.goal_pose.pose.orientation.y
        q2 = self.goal_pose.pose.orientation.z
        q3 = self.goal_pose.pose.orientation.w
        n = 2.0*(q3*q2+q0*q1)
        d = 1.0 - 2.0*(q1*q1+q2*q2)
        self.goal_heading = math.degrees( math.atan2(n,d) )
        print("Goal yaw: ", self.goal_heading)


    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose = data.pose
        cov = data.pose.covariance
        print("Current bot x: ", self.ttbot_pose.pose.position.x)
        print("Current bot y: ", self.ttbot_pose.pose.position.y)
        q0 = self.ttbot_pose.pose.orientation.x
        q1 = self.ttbot_pose.pose.orientation.y
        q2 = self.ttbot_pose.pose.orientation.z
        q3 = self.ttbot_pose.pose.orientation.w
        n = 2.0*(q3*q2+q0*q1)
        d = 1.0 - 2.0*(q1*q1+q2*q2)
        self.current_heading = math.degrees( math.atan2(n,d) )


    def callibrate(self):
        cmd_vel = Twist()
        if (not(self.current_heading>-70 and self.current_heading<-5)): #callibration will fail if we start in [-5,-70] heading
            cmd_vel.angular.z = 0.25
            self.cmd_vel_pub.publish(cmd_vel)
            print("Callibrating, please wait")
            return 1 #we are still callibrating, continue to stay in while loop
        else:
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)
            print("Callibration done")
            return 0 #we are done callibrating, exit while loop after setting yaw to 0

    def align_ttbot(self, target=0):
        cmd_vel = Twist()
        dt = 1/100
        error = (target-self.current_heading)
        if (abs(error)>5):
            PID_obj_1 = PidController(0.0007, 0.0006, 0.0001, dt, -2, 2)
            cmd_vel.angular.z = PID_obj_1.step(error)
            print("Pls wait, aligning to : ", target)
            self.cmd_vel_pub.publish(cmd_vel)
            return 1
        else:
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)
            print("Done aligning")
            return 0

    def move_ttbot(self, target_y=0, target_x=0):
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

    
    def path_follower(self,transformed_path):
        i = 0
        for y,x,heading in transformed_path:
            i = i+1
            while(self.align_ttbot(heading)):
                pass
            #'''
            while self.move_ttbot(y, x):
                if i==(len(transformed_path)):
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0
                    self.cmd_vel_pub.publish(cmd_vel)
                    return True
            #'''

    
    def a_star_path_planner(self):
        y_rviz_goal, x_rviz_goal = self.goal_pose.pose.position.y, self.goal_pose.pose.position.x
        y_rviz_current, x_rviz_current = self.ttbot_pose.pose.position.y, self.ttbot_pose.pose.position.x
        res = 0.05
        y_pgm = int(100 + (1/res)*(0-y_rviz_current))
        x_pgm = int(100 + (1/res)*(0+x_rviz_current))
        #self.end_pt = str(y_pgm) + ',' + str(x_pgm)
        self.end_pt = "85,90"
        #we should ideally pass end_pt as argument
        #because we don't know the mapping b/w pixel and gazebo data
        #we are not passing end_pt
        path = trigger_a_star(self.end_pt) 
        print(path)
        return path

    def convert_waypoints(self, path):
        '''
        transforming from pgm to gazebo waypoints
        '''
        transformed_path = []
        for index in range(len(path)-1):
            y1, x1 = path[index]
            y2, x2 = path[index+1]
            res = 0.05
            y_rviz = 0.0 - res*(y1-100)
            x_rviz = 0.0 + res*(x1-100)
            heading = -90 + math.degrees(math.atan2( (x2-x1), (y2-y1) ))
            transformed_path.append((y_rviz, x_rviz, heading))

        y_rviz = self.goal_pose.pose.position.y
        x_rviz = self.goal_pose.pose.position.x
        heading = self.goal_heading
        #transformed_path.append((y_rviz, x_rviz, heading))

        print(transformed_path)

        return transformed_path        


    def run(self):
        path_complete = False
        timeout = False
        path = self.a_star_path_planner() #call only once and get a-start solution
        transformed_path = self.convert_waypoints(path)
        while not rospy.is_shutdown():
            '''
            if self.path_follower(transformed_path) == True:
                break
            '''
            self.rate.sleep() 
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.name))


if __name__ == "__main__":
    nav = Navigation(node_name='Navigation')
    nav.init_app()

    #'''
    while (nav.callibrate()):
        pass
    while(nav.align_ttbot(0)):
        pass
    #'''

    try:
        nav.run()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)