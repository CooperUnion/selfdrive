import cv2

img = cv2.imread('yeoldtownroad.png')

# ret,bw_img = cv2.threshold(img,220,225,cv2.THRESH_BINARY)
gr_img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


# bw = cv2.threshold(img,240,255,cv2.THRESH_BINARY)

# cv2.imshow("Binary",bw_img)
cv2.imshow('Grayscale', gr_img)

# cv2.imwrite("yeoldtownroadbw.png",bw_img)
cv2.imwrite("yeoldtownroadgr.png",gr_img)

cv2.waitKey(0)
cv2.destroyAllWindows()
