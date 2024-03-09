
import cv2
import matplotlib.pyplot as plt
import numpy as np


def unwarp(img, src, dst, testing):
    h, w = img.shape[:2]
    # use cv2.getPerspectiveTransform() to get M, the transform matrix, and Minv, the inverse
    M = cv2.getPerspectiveTransform(src, dst)
    # use cv2.warpPerspective() to warp your image to a top-down view
    warped = cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)

    if testing:
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        f.subplots_adjust(hspace=.2, wspace=.05)
        ax1.imshow(img)
        x = [src[0][0], src[2][0], src[3][0], src[1][0], src[0][0]]
        y = [src[0][1], src[2][1], src[3][1], src[1][1], src[0][1]]
        ax1.plot(x, y, color='red', alpha=0.4, linewidth=3, solid_capstyle='round', zorder=2)
        ax1.set_ylim([h, 0])
        ax1.set_xlim([0, w])
        ax1.set_title('Original Image', fontsize=30)
        ax2.imshow(cv2.flip(warped, 1))
        ax2.set_title('Unwarped Image', fontsize=30)
        plt.show()
    else:
        return warped, M


if __name__ == "__main__":

    # Live camera feed
    cap = cv2.VideoCapture("/dev/video4") # Find Correct USB device for Camera Capture

    if not cap.isOpened():
            print("cannot open camera")

    # We will first manually select the source points 
    # we will select the destination point which will map the source points in
    # original image to destination points in unwarped image
    src = np.float32([(200, 200),
                      (400, 200),
                      (200, 500),
                      (400, 500)])

    #top left, top right, bottom left, bottom right
    dst = np.float32([(170, 180),
                      (430, 180),
                      (170, 530),
                      (430, 530)])

    while(True):
        # Capture frame-by-frame
        ret, img = cap.read()

        # Our operations on the img come here
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        cv2.imshow('img', gray) # Size (480, 640)
        warped, M = unwarp(gray, src, dst, False)
        cv2.imshow('Warped', warped) # Size (480, 640)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
