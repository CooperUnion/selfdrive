#!/usr/bin/env python3

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import numpy as np
from matplotlib import pyplot as plt


# DEFINES
# Subscribed Topics
IMAGE_TOPIC = '/zed/zed_node/rgb/image_rect_color'  
# rgb/image topic and depth topic come from left camera
# Depth map image registered on the left image (32-bit float in meters by default)
DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered' # something like this

#Published Topics
LANE_TOPIC = '/lane_detection/rgb/image_rect_color'

# Parameters
BLUR_PARAM_X = 9
BLUR_PARAM_Y = 9

kernel = np.ones((5,5),np.float32)/25
bridge = CvBridge()

class Subscriber():
    def __init__(self):
        
        self.cv_img = None
        rospy.Subscriber(IMAGE_TOPIC, Image, self.img2cv)

    def img2cv(self,data):
        rospy.loginfo('Video Frame Received')
        # Converts image topic stream to cv image
        # Possible encoding schemes: 8UC[1-4], 8SC[1-4], 16UC[1-4], 16SC[1-4], 32SC[1-4], 32FC[1-4], 64FC[1-4]     
        # Can optionally do color or pixel depth conversion
        self.cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        
class Publisher():
    def __init__(self):
        self.lane_pub =  rospy.Publisher(LANE_TOPIC, Image, queue_size=2)
        self.rate = rospy.Rate(1)
    def publish_lane(self,cv_image):
        img_msg = bridge.cv2_to_imgmsg(cv_image, "passthrough")
        self.lane_pub.publish(img_msg)
        self.rate.sleep()

# Clean this up later 
class Lanes():
    def __init__(self):
        self.source_img = cv.imread("../images/test.jpg") # small glitch not sure how to intialize image
        # self.source_img = None
    def detection(self):
        grayed  = cv.GaussianBlur(cv.cvtColor(self.source_img, cv.COLOR_BGR2GRAY), (BLUR_PARAM_X,BLUR_PARAM_Y),0)
        ret, thresh = cv.threshold(grayed, 200, 255, 0)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        grayed = cv.cvtColor(grayed, cv.COLOR_GRAY2BGR) 
        cv.drawContours(self.source_img, contours, -1, (0,0,255), -1)
        # cv.namedWindow("Lane Lines", cv.WINDOW_NORMAL)
        # cv.resizeWindow("Lane Lines", 1920, 1080)
        # cv.imshow("Lane Lines", self.source_img)
        # cv.waitKey(1)
        return self.source_img

    def get_largest_contour(contours, min_area: int = 30):
        """
    Finds the largest contour with size greater than min_area.

    Args:
        contours: A list of contours found in an image.
        min_area: The smallest contour to consider (in number of pixels)

    Returns:
        The largest contour from the list, or None if no contour was larger
        than min_area.
        """
    # Check that the list contains at least one contour
        if len(contours) == 0:
            return None

        # Find and return the largest contour if it is larger than min_area
        greatest_contour = max(contours, key=cv.contourArea)
        if cv.contourArea(greatest_contour) < min_area:
            return None
        return greatest_contour

def main():

    rospy.init_node('car_vision', anonymous=True)
    sub_img = Subscriber()
    pub_img = Publisher()
    lanes = Lanes()

    while not rospy.is_shutdown():
        lanes.source_img = sub_img.cv_img
        lanes_img = lanes.detection()
        pub_img.publish_lane(lanes_img)

if __name__ == "__main__":
    main()
    

