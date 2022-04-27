#!/usr/bin/env python3

import sys
from cv2 import contourArea
import rospy
import cv2
import numpy  as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import Bool

class obstacle_detector:
 
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.__image_callback)
        self.obstacle_pub = rospy.Publisher("/obstacle_status",Bool,queue_size=50)
        self.max_area = 0.0

 
    def __image_callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.get_max_area(cv_image)
        except CvBridgeError as e:
            print(e)

    def get_max_area(self, cv_image):
        sensitivity = 5
        lower_white = np.array([0,0,255-sensitivity])
        upper_white = np.array([255,sensitivity,255])
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, lower_white, upper_white)
        masked_img = cv2.bitwise_and(cv_image, cv_image, mask= mask)
        bgr_img = cv2.cvtColor(masked_img, cv2.COLOR_HSV2BGR)
        gray_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)
        ret, thresh_img = cv2.threshold(gray_img, 127, 255, 0)
        contours, hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(masked_img, contours, -1, (0,0,255), 6)
        area = 3*[0.0] #max no of dynamic obstacles = 3
        i = 0
        for c in contours:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0
            coordinate = str(cX) + ',' + str(cY)
            area[i] = cv2.contourArea(c)
            cv2.circle(masked_img, (cX, cY), 5, (0, 0, 255), -1)
            cv2.putText(masked_img, coordinate, (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 0, 0), 1)
            cv2.putText(masked_img, str(area[i]), (cX + 25, cY + 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 225, 0), 1)
            i=i+1
        self.max_area = max(area)
        print("Max area = ", self.max_area)

        safe_area = 250000.0
        obstacle_detect = Bool()
        obstacle_detect.data = False
        if self.max_area>safe_area:
            obstacle_detect.data = True
        self.obstacle_pub.publish(obstacle_detect)

        cv2.imshow("Image window", masked_img)
        cv2.waitKey(3)
    
 
def main(args):
    rospy.init_node('obstacle_detector', anonymous=True)
    od = obstacle_detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)