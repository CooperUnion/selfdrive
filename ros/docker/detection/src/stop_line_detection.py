import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import rospy
import os 
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy as np
from matplotlib import pyplot as plt
AREA_THRESH = 30 
BLUR_PARAM_X, BLUR_PARAM_Y = (13,13)

class StopLine():
    def __init__(self):
        self.source_img = cv.imread("../images/lane.jpg") # small glitch not sure how to intialize image
        # self.source_img = None

    def detection(self,img):
        strip = img.copy()
        print(strip.shape)
        strip = strip[235:250, 110:190]

        # find proportion of white pixels
        gray = cv.cvtColor(strip, cv.COLOR_BGR2GRAY)
        LANE_THRESH = 200

        num_bright = np.sum(gray > LANE_THRESH)
        num_dim = np.sum(gray <= LANE_THRESH)
        ratio = num_bright / num_dim
        print(f"bright: {num_bright}, dim: {num_dim}, ratio: {ratio}")


        print(gray.shape)

        # hsv = cv.cvtColor(strip, cv.COLOR_BGR2HSV)
        self.source_img = gray
        return ratio > 0.20

