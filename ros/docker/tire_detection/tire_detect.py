import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
import time
src = cv.imread('TIRED.jpeg')
BLUR_PARAM_X = 9
BLUR_PARAM_Y = 9
kernel = np.ones((5,5),np.float32)/25
grayed  = cv.GaussianBlur(cv.cvtColor(src, cv.COLOR_BGR2GRAY), (BLUR_PARAM_X,BLUR_PARAM_Y),0)
ret, thresh = cv.threshold(grayed, 100, 255, cv.THRESH_BINARY_INV)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
cv.drawContours(src, contours, -1, (0,0,255), -1)
RGB_img = cv.cvtColor(src, cv.COLOR_BGR2RGB)

## Display Code ##
cv.imshow("Modified", src)
plt.axis('off')
plt.imshow(RGB_img)
plt.show()
cv.waitKey(10000)