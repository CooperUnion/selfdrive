import cv2
import time


vidcap_right = cv2.VideoCapture("/dev/video1")

vidcap_left = cv2.VideoCapture("/dev/video3")

while True:
        # time.sleep(.007)
        right = vidcap_right.read()
        if(right[0]):
            cv2.imshow("right",right[1])
        # time.sleep(.007)

        left = vidcap_left.read()
        if(left[0]):
            cv2.imshow("left", left[1])

        if cv2.waitKey(10) == 27:
            break
