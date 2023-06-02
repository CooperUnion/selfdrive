import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

kernel = np.ones((5,5),np.float32)/25
BLUR_PARAM_X = 9
BLUR_PARAM_Y = 9



source_img = cv.imread('test.jpg')[1700:,1000:]
grayed  = cv.GaussianBlur(cv.cvtColor(source_img, cv.COLOR_BGR2GRAY), (BLUR_PARAM_X,BLUR_PARAM_Y),0)
ret, thresh = cv.threshold(grayed, 200, 255, 0)
cv.imwrite("thresholdret.jpg",thresh)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
grayed = cv.cvtColor(grayed, cv.COLOR_GRAY2BGR) 
cv.drawContours(source_img, contours, -1, (0,0,255), -1)
print(contours)
cv.imwrite("ColorizedContours.jpg", source_img)
cv.imwrite("grayed.jpg",grayed)
