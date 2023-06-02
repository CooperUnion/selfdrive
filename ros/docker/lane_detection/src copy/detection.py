#!/usr/bin/env python3

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import time


# DEFINES
IMAGE_TOPIC = '/zed/zed_node/rgb/image_rect_color'  # something like this
BLUR_PARAM_X = 9
BLUR_PARAM_Y = 9

kernel = np.ones((5,5),np.float32)/25
source_img = cv.imread("./test.jpg")
plt.imshow(source_img)
grayed  = cv.GaussianBlur(cv.cvtColor(source_img, cv.COLOR_BGR2GRAY), (BLUR_PARAM_X,BLUR_PARAM_Y),0)
ret, thresh = cv.threshold(grayed, 200, 255, 0)
# cv.imwrite("../images/thresholdret.jpg",thresh)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
grayed = cv.cvtColor(grayed, cv.COLOR_GRAY2BGR) 
cv.drawContours(source_img, contours, -1, (0,0,255), -1)
cv.imshow("Modified", source_img)
RGB_img = cv.cvtColor(source_img, cv.COLOR_BGR2RGB)
plt.axis('off')
plt.imshow(RGB_img)
plt.show()


cv.waitKey(20000)
cv.destroyAllWindows()

print(contours)

