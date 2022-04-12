#!/usr/bin/env python3


import sys
import math
import time
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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
        self.lidar_dist = min(data.ranges[30], data.ranges[0], data.ranges[-30]) #extracting critical laser-scan data
        #print("Lidar dist = ", self.lidar_dist)


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


    def wall_orientation(self):
        wall = '-' # - for top/bottom wall, | for right/left wall
        if (self.current_heading>45 and self.current_heading<135): #left wall
            wall = '|'
        elif (self.current_heading<-45 and self.current_heading>-135): #right wall
            wall = '|'
        return wall

    def path_follower(self):
        self.move_ttbot(0.3)
        if self.lidar_dist<1.0:
            target_heading = 180-self.current_heading
            wall = self.wall_orientation()
            if wall == '|':
                target_heading = -self.current_heading
            while not self.align_ttbot(target_heading):
                pass


    def move_ttbot(self,speed=0):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.6
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

    def align_ttbot(self, target_heading):

        cmd_vel = Twist()
        cmd_vel.linear.x = -0.01
        dt = 1/100
        flag = False
        if (target_heading>180):
            target_heading = -(360-target_heading)
        error = (target_heading-self.current_heading)
        if (abs(error)>10):
            PID_obj_1 = PidController(0.0007, 0.0006, 0.0001, dt, -1, 1)
            cmd_vel.angular.z = PID_obj_1.step(error)
            print("Pls wait, aligning to : ", target_heading)
        else:
            print("Done aligning")
            cmd_vel.angular.z = 0
            flag = True

        self.cmd_vel_pub.publish(cmd_vel)
        return flag

    def run(self):
        scan_complete = False
        timeout = False
        time.sleep(2) #just wait until everything loads
        data_points_count = 0
        while not rospy.is_shutdown():
            data_points_count = data_points_count + 1
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