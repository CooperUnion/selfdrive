import numpy as np
import cv2 as cv

def getLargestLine(frame: np.ndarray):
    #https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html 
    edge_image = cv.Canny(frame, 250, 150)
    lines = cv.HoughLinesP(
        edge_image, 1, np.pi / 180, 100, minLineLength=30, maxLineGap=250)
    maxlen = 0
    #Guarantees that even if theres no line, we're setting it to the maximum potential value,
    ##so there's no functional error
    maxlendata = [frame.shape, 0, 0, 0]
    #Iterate over all found lines
    for l in lines[:]:
        cur = l[0]
        #Don't need to take the root of this: quantized means all values are positive
        #Len2 as it's len squared
        if len2 := (cur[2] - cur[0])**2 + (cur[3] - cur[1])**2 > maxlen:
            #Walrus because why not
            maxlen = len2
            maxlendata = cur
    return maxlendata


cap = cv.VideoCapture("/dev/video0")  # check this
ret = True

# TODO: ROSSIFY THIS
# TODO: Parametrize with sliders
while ret:
    # Capture frame-by-frame
    ret, m_frame = cap.read()
    width = m_frame.shape[1] // 2
    # Splitting images here as we only have one source: In other case, this code would be same for both cameras
    # and a rosnode would compare outputs
    frames = [(m_frame[:, :width], m_frame[:, width:])]
    # Need this to persist
    errors = []
    for frame in frames:
        # Convert to hsv, perform basic image processing on HSV ALONE
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV_FULL)
        gray = cv.GaussianBlur(hsv[:, :, 2], (9, 9), 0)
        threshed = cv.threshold(gray, 200, 255, cv.THRESH_BINARY_INV)[1]

        # Get the largest lane line, on processed image, this is what will be used
        line = getLargestLine(threshed)

        # Draw the line
        cv.line(
            frame,
            (line[0], line[1]),
            (line[2], line[3]),
            (0, 0, 255),
            1,
            cv.LINE_AA,
        )
        # Writing the "error": In essence, our distance to the line .
        # This uses the average x coordinate as a reference to where the line "sits", and should not be swayed by warping.
        errors.append(abs(line[0] - line[2]) / 2)
        cv.imshow("Source with lines", frame)
    # This should be input to controller

    net_error = errors[1] - errors[2]
    print(net_error)
    # Necessary for cv to work, otherwise it gets scared (?) Don't ask it just doesn't work if this isn't here
    if cv.waitKey(1) != -1:
        break
    # When everything done, release the capture
    cap.release()
    cv.destroyAllWindows()
