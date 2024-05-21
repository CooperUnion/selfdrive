import cv2
import numpy as np
from PIL import Image
import copy


def to_png(src, target):
    img = Image.open(src)
    img.save(target)


# increase contrast of bgr img to bgr img
def inc_cont(img: np.ndarray):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l_channel, a, b = cv2.split(lab)

    # Applying CLAHE to L-channel
    # feel free to try different values for the limit and grid size:
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    cl = clahe.apply(l_channel)

    # merge the CLAHE enhanced L-channel with the a and b channel
    limg = cv2.merge((cl, a, b))

    # Converting image from LAB Color model to BGR color spcae
    enhanced_img = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)

    return enhanced_img


def hsv_filter(img: np.ndarray):
    """
    bgr img -> hsv, sat, grayscale img
    """

    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_img)

    return hsv_img, s, v


def get_oct(
        sat_img: np.ndarray,
        ori_img: np.ndarray,
        debug=False) -> np.ndarray:
    """
    input:
        sat_img (np.ndarray): saturation channel of hsv image.
        ori_img (np.ndarray): original image on which the octagons will be drawn.
        debug (boolean): whether to display the image.
    return:
        a binary mask of the largest octagon in the image.
        Inside of the octagon is (255, 255, 255) and the
        outside is (0, 0, 0).
    """
    # Add Gaussian Blur to the image
    gauss = cv2.GaussianBlur(sat_img, (7, 7), 0)

    # Make a binary image
    _, gauss_blur_binary_img = cv2.threshold(gauss, 90, 255, cv2.THRESH_BINARY)

    # Find the contours
    contours, _ = cv2.findContours(
        gauss_blur_binary_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if debug:
        all_oct_img = copy.deepcopy(ori_img)

    # Prepare lists to get the largest octagon
    length_list = []
    approx_list = []

    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        if len(approx) == 8:
            # Compute the summed length of the edges
            points = np.vstack((cnt, np.expand_dims(cnt[0], 0)))
            length = np.sum(np.linalg.norm(points[1:] - points[:-1], axis=2))
            length_list.append(length)
            approx_list.append(approx)

            if debug:
                cv2.drawContours(all_oct_img, [approx], -1, (0, 255, 255), 3)

    if debug:
        cv2.imshow('octagon detected', all_oct_img)
        cv2.waitKey(1)

    if len(length_list) == 0:
        return False

    # Find the contour with longest sum of length of edges
    idx = np.where(length_list == max(length_list))[0][0]
    the_octagon = approx_list[idx].reshape(-1, 2)

    # Create a binary mask
    mask = np.zeros(sat_img.shape)
    cv2.fillPoly(mask, np.int32([the_octagon]), color=(255, 255, 255))
    mask = np.array(mask, dtype=np.uint8)

    return mask


def dilate(img):
    src = img
    kernel = np.ones((3, 3))
    # You can try more different parameters
    dst = cv2.dilate(src, kernel)

    return dst


def red_mask(hsv_img: np.ndarray):
    result = hsv_img.copy()

    lower = np.array([20, 130, 63])
    upper = np.array([200, 255, 255])
    mask = cv2.inRange(hsv_img, lower, upper)
    result = cv2.bitwise_and(result, result, mask=mask)

    return mask, result
