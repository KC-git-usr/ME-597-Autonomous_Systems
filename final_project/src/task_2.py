#!/usr/bin/env python3

from cgi import test
import sys
import os

import math
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist

from Controller import PidController
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


    def init_app(self):
        rospy.init_node(self.node_name, anonymous=True)
        self.rate = rospy.Rate(100)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.__goal_pose_cbk, queue_size=50)
        rospy.Subscriber('/odom', Odometry, self.__ttbot_pose_cbk, queue_size=50)
        self.path_pub = rospy.Publisher('global_plan', Path, queue_size=50)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=50)


    def __goal_pose_cbk(self, data):
        self.goal_pose = data
        q0 = self.goal_pose.pose.orientation.x
        q1 = self.goal_pose.pose.orientation.y
        q2 = self.goal_pose.pose.orientation.z
        q3 = self.goal_pose.pose.orientation.w
        n = 2.0*(q3*q2+q0*q1)
        d = 1.0 - 2.0*(q1*q1+q2*q2)
        self.goal_heading = math.degrees( math.atan2(n,d) ) #output yaw is in degrees


    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose = data.pose
        cov = data.pose.covariance
        q0 = self.ttbot_pose.pose.orientation.x
        q1 = self.ttbot_pose.pose.orientation.y
        q2 = self.ttbot_pose.pose.orientation.z
        q3 = self.ttbot_pose.pose.orientation.w
        n = 2.0*(q3*q2+q0*q1)
        d = 1.0 - 2.0*(q1*q1+q2*q2)
        self.current_heading = math.degrees( math.atan2(n,d) )  #output yaw is in degrees


    def callibrate(self):
        cmd_vel = Twist()
        if (not(self.current_heading>-70 and self.current_heading<-5)): #callibration will fail if we start in [-5,-70] heading
            cmd_vel.angular.z = 0.25
            self.cmd_vel_pub.publish(cmd_vel)
            return 1 #we are still callibrating, continue to stay in while loop
        else:
            cmd_vel.angular.z = 0
            self.cmd_vel_pub.publish(cmd_vel)
            return 0 #we are done callibrating, exit while loop after setting yaw to 0

    def align_ttbot(self, target=0):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        flag = True
        dt = 1/100
        error = (target-self.current_heading)
        if (abs(error)>5):
            PID_obj_1 = PidController(0.0007, 0.0006, 0.0001, dt, -2, 2)
            cmd_vel.angular.z = PID_obj_1.step(error)
            print("Pls wait, aligning to : ", target)
        else:
            cmd_vel.angular.z = 0
            print("Done aligning")
            flag = False

        self.cmd_vel_pub.publish(cmd_vel)
        return flag
        

    def align_test(self, target=0):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        dt = 1/100
        error = (target-self.current_heading)
        print("Pls wait, aligning to : ", target)
        while abs(error)>0.5:
            error = (target-self.current_heading)
            PID_obj_1 = PidController(0.0009, 0.0008, 0.0001, dt, -2, 2)
            cmd_vel.angular.z = PID_obj_1.step(error)
            self.cmd_vel_pub.publish(cmd_vel)

        cmd_vel.angular.z = 0
        print("Done aligning")
        self.cmd_vel_pub.publish(cmd_vel)
        

    def move_ttbot(self, target_y=0, target_x=0):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0
        flag = True
        dt = 1/100
        d = ( (target_y-self.ttbot_pose.pose.position.y)**2 + (target_x-self.ttbot_pose.pose.position.x)**2)**(0.5)
        error = abs(d)
        if (error>0.15):
            PID_obj_2 = PidController(0.1, 0.008, 0.001, dt, 0.0, 2)
            cmd_vel.linear.x = PID_obj_2.step(error)
            print("Moving to : ", target_y, target_x)
        else:
            cmd_vel.linear.x = 0
            print("Vel={} error={} d={} t_y={} t_x={} c_y={} c_x+{}".format(cmd_vel.linear.x, error, d, target_y, target_x, \
                self.ttbot_pose.pose.position.y, self.ttbot_pose.pose.position.x))
            flag = False

        self.cmd_vel_pub.publish(cmd_vel)
        return flag


    def move_test(self, target_y=0, target_x=0):
        cmd_vel = Twist()
        cmd_vel.angular.z = 0
        dt = 1/100
        d = ( (target_y-self.ttbot_pose.pose.position.y)**2 + (target_x-self.ttbot_pose.pose.position.x)**2)
        error = abs(d)
        print("Moving to : ", target_y, target_x)
        while error>0.05:
            d = ( (target_y-self.ttbot_pose.pose.position.y)**2 + (target_x-self.ttbot_pose.pose.position.x)**2)
            error = abs(d)
            PID_obj_2 = PidController(0.01, 0.009, 0.008, dt, 0.0, 0.4)
            cmd_vel.linear.x = PID_obj_2.step(error)
            self.cmd_vel_pub.publish(cmd_vel)

        cmd_vel.linear.x = 0
        print("Vel={} error={} d={} t_y={} t_x={} c_y={} c_x+{}".format(cmd_vel.linear.x, error, d, target_y, target_x, \
        self.ttbot_pose.pose.position.y, self.ttbot_pose.pose.position.x))
        self.cmd_vel_pub.publish(cmd_vel)


    def path_follower(self,transformed_path):
        i = 0
        for y,x,heading in transformed_path:
            self.align_test(heading)
            self.move_test(y, x)

    
    def a_star_path_planner(self):
        y_rviz_goal, x_rviz_goal = self.goal_pose.pose.position.y, self.goal_pose.pose.position.x
        y_rviz_current, x_rviz_current = self.ttbot_pose.pose.position.y, self.ttbot_pose.pose.position.x
        res = 0.1005

        y_pgm = int(192 - (1/res)*(10+y_rviz_current))
        x_pgm = int(-0 + (1/res)*(10+x_rviz_current))
        start_pt = str(y_pgm) + ',' + str(x_pgm)

        y_pgm = int(192 - (1/res)*(10+y_rviz_goal))
        x_pgm = int(-0 + (1/res)*(10+x_rviz_goal))
        end_pt = str(y_pgm) + ',' + str(x_pgm)

        path = trigger_a_star(start_pt, end_pt) 

        return path

    def path_follower_test(self,path):
        for index in range(len(path)):
            res = 0.1005
            y, x = path[index]
            y_rviz = -10.0 + res*(192-y)
            x_rviz = -10.0 + res*(0+x)
            heading = math.degrees(math.atan2( (y_rviz-self.ttbot_pose.pose.position.y), (x_rviz-self.ttbot_pose.pose.position.x) ))
            self.align_test(heading)
            self.move_test(y_rviz, x_rviz)
        heading = self.goal_heading
        self.align_test(heading)

    def convert_waypoints(self, path):
        '''
        transforming from pgm to gazebo waypoints
        '''
        transformed_path = []
        for index in range(len(path)-1):
            y1, x1 = path[index]
            y2, x2 = path[index+1]
            res = 0.1005
            y_rviz_1 = -10.0 + res*(192-y1)
            x_rviz_1 = -10.0 + res*(0+x1)
            y_rviz_2 = -10.0 + res*(192-y2)
            x_rviz_2 = -10.0 + res*(0+x2)
            heading = math.degrees(math.atan2( (y_rviz_2-y_rviz_1), (x_rviz_2-x_rviz_1) ))
            transformed_path.append((y_rviz_1, x_rviz_1, heading))

        y_rviz = self.goal_pose.pose.position.y
        x_rviz = self.goal_pose.pose.position.x
        heading = self.goal_heading
        transformed_path.append((y_rviz, x_rviz, heading))

        print("transformed_path = ", transformed_path)

        return transformed_path        


    def run(self):
        path_complete = False
        timeout = False
        path = self.a_star_path_planner() #call only once and get a-start solution
        #transformed_path = self.convert_waypoints(path)
        #self.path_follower(transformed_path)
        self.path_follower_test(path)
        print("Goal reached")
        while not rospy.is_shutdown():
            self.rate.sleep() 
        rospy.signal_shutdown("[{}] Finished Cleanly".format(self.name))


if __name__ == "__main__":
    nav = Navigation(node_name='Navigation')
    nav.init_app()

    print("Callibrating, please wait")
    while (nav.callibrate()):
        pass
    nav.align_test(0)
    print("Callibration done")

    try:
        nav.run()
        # nav.align_test(-90)
        # nav.move_test(-1.0,-1.0)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)