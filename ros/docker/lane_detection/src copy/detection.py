#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import time

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

def get_contour_center(contour):
    """
    Finds the center of a contour from an image.
    Args:
        contour: The contour of which to find the center.
    Returns:
        The (row, column) of the pixel at the center of the contour, or None if the
        contour is empty.
    """
    M = cv.moments(contour)
    # Check that the contour is not empty
    # (M["m00"] is the number of pixels in the contour)
    if M["m00"] <= 0:
        return None
    # Compute and return the center of mass of the contour
    center_row = round(M["m01"] / M["m00"])
    center_column = round(M["m10"] / M["m00"])
    return (center_row, center_column)




# DEFINES
IMAGE_TOPIC = '/zed/zed_node/rgb/image_rect_color'  # something like this
BLUR_PARAM_X = 9
BLUR_PARAM_Y = 9
AREA_THRESH = 150 ## Thresholding for minimum area

source_img = cv.imread("docker/lane_detection/src copy/test.jpg")
grayed  = cv.GaussianBlur(cv.cvtColor(source_img, cv.COLOR_BGR2GRAY), (BLUR_PARAM_X,BLUR_PARAM_Y),0)
ret, thresh = cv.threshold(grayed, 200, 255, 0)
# cv.imwrite("../images/thresholdret.jpg",thresh)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
grayed = cv.cvtColor(grayed, cv.COLOR_GRAY2BGR)
for contour in contours:
    area = cv.contourArea(contour)
    # contour = get_largest_contour(contour)
    if area > AREA_THRESH:
        (x0,y0) = get_contour_center(contour)
        cv.circle(source_img, (y0,x0), radius=20, color=(255,0,0), thickness=10)
        cv.drawContours(source_img, contour, -1, (0,0,255), 10)

# w = np.shape(source_img)[0]
# for contour in contours:
#     area = cv.contourArea(contour)
#     if area > AREA_THRESH:
#         [vx,vy,x0,y0] = cv.fitLine(contour,cv.DIST_L2,0,0.01,0.01)
#         print (vx,vy,x0,y0)
#         cv.line(source_img,(x0*w,y0*w),((vx+x0)*w, (vy+y0)*w),(0,255,0),9)

# print(contours)
cv.namedWindow("Lane Lines", cv.WINDOW_NORMAL)
cv.resizeWindow("Lane Lines", 1920, 1080)
cv.imshow("Lane Lines", source_img)
cv.waitKey(10000)
cv.destroyAllWindows()

