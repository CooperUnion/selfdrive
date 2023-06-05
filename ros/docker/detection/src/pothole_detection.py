import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import rospy
import os 
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
from matplotlib import pyplot as plt

# DEFINES
AREA_THRESH = 5000 
BLUR_PARAM_X, BLUR_PARAM_Y = (13,13)
HUE_THRESHOLD = 200 # Before dark, 200 worked well

# Subscribed Topics
IMAGE_TOPIC = '/zed/zed_node/rgb/image_rect_color'  

# Published Topics
POT_TOPIC = '/pothole_detection/rgb/image_rect_color'

bridge = CvBridge()

class Subscriber():
    def __init__(self):
        self.cv_img = cv.imread("../images/lane.jpg")   # kinda silly
        rospy.Subscriber(IMAGE_TOPIC, Image, self.img2cv)

    def img2cv(self,data):
        # rospy.loginfo('Video Frame Received')
        # Converts image topic stream to cv image
        # Possible encoding schemes: 8UC[1-4], 8SC[1-4], 16UC[1-4], 16SC[1-4], 32SC[1-4], 32FC[1-4], 64FC[1-4]     
        # Can optionally do color or pixel depth conversion
        self.cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

class Publisher():
    def __init__(self):
        self.pot_pub =  rospy.Publisher(POT_TOPIC , Image, queue_size=2)
        self.rate = rospy.Rate(100)
    def pub_pot(self,cv_image):
        img_msg = bridge.cv2_to_imgmsg(cv_image, "passthrough")
        self.pot_pub.publish(img_msg)
        self.rate.sleep()

class Pothole():
    def __init__(self):
        self.source_img = cv.imread("../images/pot3.jpeg") # small glitch not sure how to intialize image

    def find_circle(self, contours):
        contour_list = []
        for contour in contours:
            approx = cv.approxPolyDP(contour,0.01*cv.arcLength(contour,True),True)
            area = cv.contourArea(contour)
            if ((len(approx) > 8) & (area > AREA_THRESH) ):
                contour_list.append(contour)
        return contour_list

    def detection(self,img):
        source_img = img.copy()
        self.source_img = source_img

        grayed  = cv.GaussianBlur(cv.cvtColor(self.source_img, cv.COLOR_BGR2GRAY), (BLUR_PARAM_X,BLUR_PARAM_Y),0)
        ret, thresh = cv.threshold(grayed, HUE_THRESHOLD, 255, 0)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        grayed = cv.cvtColor(grayed, cv.COLOR_GRAY2BGR) 

        contours = self.find_circle(contours)
        cv.drawContours(self.source_img, contours, -1, (0,0,255), -1)

def main():

    rospy.init_node('pothole_detection', anonymous=True)
    sub = Subscriber()
    pub = Publisher()
    pothole = Pothole()
    while not rospy.is_shutdown():
        pothole.detection(sub.cv_img)
        pub.pub_pot(pothole.source_img)

if __name__ ==  '__main__':
    main()
