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

class Lanes():
    def __init__(self):
        self.source_img = cv.imread("../images/lane.jpg") # small glitch not sure how to intialize image
        # self.source_img = None
    

    def get_largest_contour(contours, min_area: int = 30):
        # Check that the list contains at least one contour
        if len(contours) == 0:
            return None
        # Find and return the largest contour if it is larger than min_area
        greatest_contour = max(contours, key=cv.contourArea)
        if cv.contourArea(greatest_contour) < min_area:
            return None
        return greatest_contour


    def get_contour_centers(contours):
        return_arr = []
        for contour in contours:
            if cv.contourArea(contour) > AREA_THRESH:
                M = cv.moments(contour)
                # Check that the contour is not empty
                # (M["m00"] is the number of pixels in the contour)
                center_row = round(M["m01"] / M["m00"])
                center_column = round(M["m10"] / M["m00"])
                return_arr.append([center_column,center_row])
                # Compute and return the center of mass of the contour
        return np.array(return_arr)


            
    def compute_target(coords):
        final_coords = []
        if len(coords) >= 2:
            final_coords[0] = coords[0]
            final_coords[1] = coords[1]

            for (x0,y0) in coords:
                if y0 > final_coords[0][1]:
                    final_coords[0] == [x0,y0]
                elif y0 > final_coords[1][1]:
                    final_coords[1] = [x0][y0]
            return ((final_coords[0][0] + final_coords[1][0]) // 2)
        else:
            return final_coords


    def target_offset(x, img):
        height, width, channels = img.shape
        offset = (x - width//2)
        return offset


    def draw_center_points(self,img,contours):
        center_coords = self.get_contour_centers(contours)
        for (y0,x0) in center_coords:
            cv.circle(img, (x0,y0), radius=10, color=(255,0,0), thickness=10)
            cv.drawContours(img, contours, -1, (0,0,255), 10)
        return img
    

    def detection(self,img):
        source_img = img.copy()
        self.source_img = source_img

        grayed  = cv.GaussianBlur(cv.cvtColor(self.source_img, cv.COLOR_BGR2GRAY), (BLUR_PARAM_X,BLUR_PARAM_Y),0)
        ret, thresh = cv.threshold(grayed, 200, 255, 0)
        contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        grayed = cv.cvtColor(grayed, cv.COLOR_GRAY2BGR) 
        contour_img = cv.drawContours(self.source_img, contours, -1, (0,0,255), -1)
        self.draw_center_points(contours)

        #offset Determination

        center_coordinates = self.get_contour_center(contours)
        target_coordinate = self.compute_target(center_coordinates)
        if (len(target_coordinate) != 0):
            offset = self.target_offset(target_coordinate,source_img)
            return offset


