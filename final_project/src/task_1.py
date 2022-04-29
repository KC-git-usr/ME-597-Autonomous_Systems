#!/usr/bin/env python3


from collections import deque
from dataclasses import dataclass
import sys
import math
import time
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from Controller import PidController

class Navigation:


    def __init__(self, node_name="Auto_mapping"):
        # ROS related variables
        self.node_name = node_name
        self.rate = 0
        # Path planner/follower related variables
        self.ttbot_pose = Odometry()
        self.current_heading = 0
        self.lidar_dist = 0
        self.lidar_data = LaserScan()
        self.last_four_pts = deque()


    def init_app(self):
        # ROS node initilization  
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(50)
        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.__scan_callback)
        rospy.Subscriber('/odom', Odometry, self.__ttbot_pose_cbk, queue_size=1)
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    

    def __scan_callback(self, data):
        self.lidar_data = data
        self.lidar_dist = min(data.ranges[30], data.ranges[0], data.ranges[330]) #extracting critical laser-scan data


    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose = data.pose
        cov = data.pose.covariance
        q0 = self.ttbot_pose.pose.orientation.x
        q1 = self.ttbot_pose.pose.orientation.y
        q2 = self.ttbot_pose.pose.orientation.z
        q3 = self.ttbot_pose.pose.orientation.w
        n = 2.0*(q3*q2+q0*q1)
        d = 1.0 - 2.0*(q1*q1+q2*q2)
        self.current_heading = math.degrees( math.atan2(n,d) )


    def format_angle(self, angle):
        if angle>180:
            angle = -(360-angle)
        if angle<-180:
            angle = (360+angle)
        return angle


    def get_out(self):
        print("Get out trigerred")
        angles = [0, 30, 60, 90, 120, 180, 210, 250, 270, 300, 330]
        best_angle = 0
        for angle in angles:
            if self.lidar_data.ranges[angle]>=self.lidar_data.ranges[best_angle]:
                best_angle = angle  

        target_heading = self.current_heading+best_angle

        print("Pls wait, aligning to : ", target_heading)
        while self.align_ttbot(target_heading):
            pass
        print("Done aligning")


    def wall_orientation(self):
        wall = '-' # - for top/bottom wall, | for right/left wall
        curr_heading = self.format_angle(self.current_heading)
        if (curr_heading<0 and curr_heading>-90): #top or right wall
            if(self.lidar_data.ranges[30]>self.lidar_data.ranges[330]): #right wall
                wall = '|'
                print("Right wall")
            else:
                wall = '-'
                print("Top wall")
        elif (curr_heading<90 and curr_heading>0): #top or left wall
            if(self.lidar_data.ranges[30]>self.lidar_data.ranges[330]): #top wall
                wall = '-'
                print("Top wall")
            else:
                wall = '|'
                print("Left wall")
        elif (curr_heading<0 and curr_heading<-90): #bottom or right wall
            if(self.lidar_data.ranges[30]>self.lidar_data.ranges[330]): #bottom wall
                wall = '-'
                print("Bottom wall")
            else:
                wall = '|'
                print("Right wall")
        elif (curr_heading>0 and curr_heading>90): #bottom or left wall
            if(self.lidar_data.ranges[30]<self.lidar_data.ranges[330]): #bottom wall
                wall = '-'
                print("Bottom wall")
            else:
                wall = '|'
                print("Left wall")
        return wall


    def path_follower(self):

        self.move_ttbot(1.5)

        if self.lidar_dist<3.0:

            y, x = round(self.ttbot_pose.pose.position.y, 1), round(self.ttbot_pose.pose.position.x, 1)
            self.last_four_pts.append((y, x))
            if len(self.last_four_pts)>4:
                self.last_four_pts.popleft()
            if (len(self.last_four_pts)==4) and (self.last_four_pts[0] == self.last_four_pts[3]):
                self.get_out()
                return

            target_heading = 180-self.current_heading
            wall = self.wall_orientation()
            if wall == '|':
                target_heading = -self.current_heading

            target_heading = self.format_angle(target_heading)
            print("Pls wait, aligning to : ", target_heading)
            print("y, x = ", y, x)
            while self.align_ttbot(target_heading):
                pass
            print("Done aligning")


    def move_ttbot(self,speed):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.0
        cmd_vel.linear.x = 0.0
        while cmd_vel.linear.x <=speed and self.lidar_dist>3.0:
            time.sleep(0.5)
            cmd_vel.linear.x += 0.1
            self.cmd_vel_pub.publish(cmd_vel)
        self.cmd_vel_pub.publish(cmd_vel)


    def align_ttbot(self, target_heading=0):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        dt = 1/100
        flag = True
        target_heading = self.format_angle(target_heading)
        curr_heading = self.format_angle(self.current_heading)
        error = (target_heading-curr_heading)
        if (abs(error)>5):
            PID_obj_1 = PidController(0.009, 0.001, 0.0003, dt, -2, 2)
            cmd_vel.angular.z = PID_obj_1.step(error)
        else:
            cmd_vel.angular.z = 0
            flag = False

        self.cmd_vel_pub.publish(cmd_vel)
        return flag


    def run(self):
        scan_complete = False
        timeout = False
        time.sleep(5) #just wait until everything loads
        while not rospy.is_shutdown():
            self.path_follower()
            self.rate.sleep()
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.name))


if __name__ == "__main__":
    nav = Navigation(node_name="Auto_mapping")
    nav.init_app()
    try:
        nav.run()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)