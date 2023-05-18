import cv2
import numpy as np

img = cv2.imread('skewmap.png')

# ret,bw_img = cv2.threshold(img,220,225,cv2.THRESH_BINARY)
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

blur = cv2.GaussianBlur(gray, (5,5), 0)

# cv2.imshow(blur, cmap='img_gr')

cv2.imshow('Gauss Blur', blur)

# cv2.imwrite("yeoldtownroadgauss.png",blur)

edges = cv2.Canny(img,100,200)

cv2.imshow('Edge', edges)

# cv2.imwrite("yeoldtownroadedge.png",edges)

# mask_white = cv2.inRange(img, 200, 255)
# mask_img = cv2.bitwise_and(gray, mask_white)

# cv2.imshow('mask_white', mask_white)

cv2.waitKey(0)
cv2.destroyAllWindows()