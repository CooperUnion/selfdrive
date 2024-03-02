import numpy as np
import cv2 as cv

AREA_THRESH = 150
cap = cv.VideoCapture("/dev/video0") # check this
ret = True
while(ret): 
    # Capture frame-by-frame
    ret, frame = cap.read()
    hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV_FULL)
    # Display the resulting frame
    gray = cv.GaussianBlur(hsv[:,:,2],(9,9),0)
    ret2, threshed = cv.threshold(gray,200,255,cv.THRESH_BINARY_INV)
    edge_image = cv.Canny(threshed,250,150)
    lines = cv.HoughLinesP(edge_image, 1, np.pi/180, 150, minLineLength=30, maxLineGap=250)
    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            cv.line(frame, (l[0], l[1]), (l[2], l[3]), (0,0,255), 1, cv.LINE_AA)
    cv.imshow("Source with lines", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break


print("Broken")
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()