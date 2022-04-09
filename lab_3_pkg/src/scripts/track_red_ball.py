#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from gazebo_msgs.msg import ModelState
from pynput import keyboard
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
import numpy  as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist


def show_img(im,gray=False):
    fig, ax = plt.subplots(figsize=(10,10))
    if gray:
        ax.imshow(im,cmap='gray')
    else:
        ax.imshow(im)


cX = 0
cY = 0
cnt_0_x = 0
cnt_0_y = 0


def image_callback(data):
    # data will contain the message information of type LaserScan, we can access and print that data as follows.
    # frame = bridge.imgmsg_to_cv2(data, "bgr8")
        global cX
        global cY
        global cnt_0_x
        global cnt_0_y

        # process image & convert to ROS
        bridge = CvBridge()
        cv2img = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        lower_red = np.array([0,100,20])    # mb_marker_buoy_red
        upper_red = np.array([10,255,255]) # world0 = sunny
        hsv_img = cv2.cvtColor(cv2img, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv_img, lower_red, upper_red)
        res_red = cv2.bitwise_and(cv2img, cv2img, mask= mask_red)           # to find the red blob
        
        # Calculate centroid of the blob of binary image using ImageMoments

        m_red = cv2.moments(mask_red, False)
        try:
            blob_area_red = m_red['m00']
        except ZeroDivisionError:
            blob_area_red = 0
        print("red:", blob_area_red)  # min: 11730    can see : 448035
        if m_red["m00"]:
            cX = int(m_red["m10"] / m_red["m00"])
            cY = int(m_red["m01"] / m_red["m00"])
        print(cX)
        print(cY)
        
        'SHOWS MASK AND ORIGINAL IMG'
        
        # cv2.imshow("RED", mask_red)  
        # cv2.imshow("Original", cv2img)
        cv2.waitKey(1)

        if cnt_0_x == 0 :
            cnt_0_x = cX
            cnt_0_y = cY

    
def trackball():
    global cX
    global cY
    global cnt_0_x
    global cnt_0_y

    # Declare the node, and register it with a unique name
    rospy.init_node('trackball', anonymous=True)
    # Define a subscriber that will catch the messages published by scan
    # Observe how the subscription has to match use some of the parameters defined for the publisher.
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    
    # This node doesn't have to run all the time, but whenever a message is received, therefore, we can leave it spinning (waiting to wake up whenever a message is available).
    # rospy.spin()
 
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
            vel_msg = Twist()
            cXz = cnt_0_x
            cYz = cnt_0_y
            # print("red2:", cXz)
            if cX < cXz:
                
                vel_msg.linear.x = 0.2

                vel_msg.angular.z = 0.2


            elif cX > cXz:
                vel_msg.linear.x = 0.2

                vel_msg.angular.z = -0.2

            # if cY < cYz:
                
            #     vel_msg.linear.x = 0.2

            #     vel_msg.angular.z = 0.0



            if cX == cXz & cY == cYz:
                vel_msg.linear.x = -0.0

                vel_msg.angular.z = -0.0      

            pub.publish(vel_msg)
            rate.sleep()
            
if __name__ == '__main__':
    try:
        trackball()
    except rospy.ROSInterruptException:
        pass




'''
Debugging:

$ rostopic info /camera/rgb/image_raw
Type: sensor_msgs/Image

Publishers: 
 * /gazebo (http://kc-Predator-G3-571:34219/)

Subscribers: 
 * /trackball (http://kc-Predator-G3-571:40679/)


$ rosmsg info sensor_msgs/Image 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data


'''