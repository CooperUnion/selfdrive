import numpy as np
import cv2 as cv

AREA_THRESH = 150
cap = cv.VideoCapture("/dev/video0") # check this
ret = True

def getLargestLine(frame: np.ndarray):
        edge_image = cv.Canny(img,250,150)
        lines = cv.HoughLinesP(edge_image, 1, np.pi/180, 100, minLineLength=30, maxLineGap=250)
        maxlen = 0
        maxlendata = [0,0,0,0]
        for l in lines[:]:
             cur = l[0]
             len2 = (cur[2] - cur[0])^2 + (cur[3] - cur[1])^2
             if len2 > maxlen:
                  maxlen = len2
                  maxlendata = cur
        return maxlendata


#TODO: ROSSIFY THIS 
while(ret): 
    # Capture frame-by-frame
    ret, m_frame = cap.read()
    width = m_frame.shape[1]//2
    frames = [(m_frame[:,:width], m_frame[:,width:])]
    errors = []
    for frame in frames:
        # Convert to hsv, perform processing on HSV
        hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV_FULL)
        gray = cv.GaussianBlur(hsv[:,:,2],(9,9),0)
        threshed = cv.threshold(gray,200,255,cv.THRESH_BINARY_INV)[1]
        #Splitting images here as we only have one source: In other case, this code would be same for both cameras
        # and a rosnode would compare outputs
        width = ret.shape[1]//2
        line = getLargestLine(threshed)
        cv.line(frame, (line[0],line[1]),(line[2],line[3]),(0,0,255), 1, cv.LINE_AA)
        errors.append(abs(line[0] - line[2])/2)
        cv.imshow("Source with lines", frame)
    net_error = errors[1] - errors[2]
    print(net_error)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()

