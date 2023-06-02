import cv2
import numpy as np
from PIL import Image
import copy

def to_png(src, target):
    img = Image.open(src)
    img.save(target)


#increase contrast of bgr img to bgr img
def inc_cont(img:np.ndarray):
    lab= cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l_channel, a, b = cv2.split(lab)

    # Applying CLAHE to L-channel
    # feel free to try different values for the limit and grid size:
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    cl = clahe.apply(l_channel)

    # merge the CLAHE enhanced L-channel with the a and b channel
    limg = cv2.merge((cl,a,b))

    # Converting image from LAB Color model to BGR color spcae
    enhanced_img = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

    return enhanced_img

def gauss_blur(img:np.ndarray):
    gauss = cv2.GaussianBlur(img, (7,7), 0)

    return gauss

def gray_binary(img:np.ndarray):
    ret,thresh = cv2.threshold(img,127,255,cv2.THRESH_BINARY)
    gaussthresh = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)
    return gaussthresh


def hsv_filter(img: np.ndarray):
    """
    bgr img -> hsv, sat, grayscale img
    """

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_img)

    return hsv_img, s, v


def get_oct(img:np.ndarray, gray:np.ndarray):
    """
    Takes in an original image (in BGR scale) and a greyscale image.
    The greyscale image is used for the octagon detection.
    Returns an original image with an octagon drawn on it.
    """
    mask = np.zeros(img.shape)

    all_oct_img = copy.deepcopy(img)

    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #ret, thresh = cv2.threshold(gray, 50, 255, 0)
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    print("Num contours detected:", len(contours))
    
    length_list = []
    approx_list = []
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt, True), True)
        
        if len(approx) == 8:
            (x, y)=cnt[0, 0]

            # Compute the summed length of the edges
            points = np.vstack((cnt, np.expand_dims(cnt[0], 0)))
            length = np.sum(np.linalg.norm(points[1:] - points[:-1], axis=2))
            length_list.append(length)
            approx_list.append(approx)

            cv2.drawContours(all_oct_img, [approx], -1, (0, 255, 255), 3)
    
    cv2.imwrite('/app/ocr/data/all_oct.png', all_oct_img)

    # Find the contour with longest sum of length of edges
    idx = np.where(length_list == max(length_list))[0][0]
    the_octagon = approx_list[idx].reshape(-1, 2)
    print(f'Best contour:\n{the_octagon}')
    
    cv2.fillPoly(mask, np.int32([the_octagon]), color = (255, 255, 255))
    mask = np.array(mask, dtype=np.uint8)

    return mask

def get_red(img:np.ndarray):
    """
    Use HSV image to get red mask
    """

    result= img.copy()
    #cv2.imwrite("/app/ocr/data/hsv2.png", result)
    lower = np.array([155,25,0])
    upper = np.array([179,255,255])
    #gauss = cv2.GaussianBlur(img, (7,7), 0)
    mask = cv2.inRange(img, lower, upper)
    cv2.imwrite("/app/ocr/data/hsv3.png", mask)
    #result = cv2.bitwise_and(result, result, mask=mask)
    return mask

    # # Apply Otsu's thresholding to the value channel
    # _, thres_img = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # # Find the red region in the hue channel
    # lower = np.array([155,25,0])
    # upper = np.array([179,255,255])
    # red_mask = cv2.inRange(hsv_img, lower, upper)
    # cv2.imwrite('/app/ocr/data/redmask.png', red_mask)

    # # Apply the red mask to the thresholded image
    # masked_img = cv2.bitwise_and(thres_img, thres_img, mask=red_mask)

def erosion(img):
    src = img
    kernel = np.ones((3,3))
    # You can try more different parameters
    dst = cv2.dilate(src, kernel)

    return dst