import cv2

vidcap_left = cv2.VideoCapture("/dev/video3")
vidcap_left.set(3,640)
vidcap_left.set(4,480)


while True:
    left = vidcap_left.read()
    if(left[0]):
        cv2.imshow("left", left[1])
    if cv2.waitKey(10) == 27:
        break
