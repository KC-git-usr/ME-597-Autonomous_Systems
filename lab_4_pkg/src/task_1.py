#!/usr/bin/env python3

import sys
import math
import random
import rospy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from sensor_msgs.msg import LaserScan

from Controlllers import PidController

class Navigation:

    def __init__(self, node_name="Auto_mapping"):
        # ROS related variables
        self.node_name = node_name
        self.rate = 0
        # Path planner/follower related variables
        self.path = Path()
        self.ttbot_pose = PoseStamped()
        self.current_heading = 0
        self.lidar_dist = 0


    def init_app(self):

        # ROS node initilization
        
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(10)
        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.__scan_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__ttbot_pose_cbk, queue_size=1)
        # Publishers
        self.path_pub = rospy.Publisher('global_plan', Path, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    def __scan_callback(self, data):
        self.lidar_dist = data.ranges[0] #extracting only front laser-scan data

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

    def gen_local_goal_pose(self):
        radius = math.sqrt(3)
        random_heading = random.uniform(0, 2*math.pi)
        goal_pose_x = self.ttbot_pose.pose.position.x + radius*math.cos(random_heading)
        goal_pose_y = self.ttbot_pose.pose.position.y + radius*math.sin(random_heading)
        goal_pose_heading = random_heading
        return goal_pose_y, goal_pose_x, goal_pose_heading

    def path_follower(self):
        reflected_angle = 180-self.current_heading
        while(self.lidar_dist>0.3):    
            self.move_ttbot(2)
        self.align_ttbot(reflected_angle)

    def move_ttbot(self,speed=0):
        cmd_vel = Twist()
        cmd_vel.linear.x = speed
        cmd_vel.angular.z = 0
        self.cmd_vel_pub.publish(cmd_vel)

    def align_ttbot(self, target_heading):
        if target_heading==0 or target_heading==180:
            target_heading = random.randint(-179,179)
        cmd_vel = Twist()
        dt = 1/100
        error = (target_heading-self.current_heading)
        if (abs(error)>5):
            PID_obj_1 = PidController(0.0007, 0.0006, 0.0001, dt, -2, 2)
            cmd_vel.angular.z = PID_obj_1.step(error)
            print("Pls wait, aligning to : ", target_heading)
        else:
            cmd_vel.angular.z = 0
            print("Done aligning")
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        scan_complete = False
        timeout = False
        data_points_count = 0
        while not rospy.is_shutdown():
            data_points_count = data_points_count + 1
            self.path_follower
            self.rate.sleep() 
            print("Reached run while loop")
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.name))


if __name__ == "__main__":
    nav = Navigation(node_name="Auto_mapping")
    nav.init_app()
    print("Reached main")
    try:
        print("Reached try")
        nav.run()
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)