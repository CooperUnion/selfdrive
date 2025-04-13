import toml
import cv2

# import numpy as np


class LaneFollower:

    def __init__(
        self, odom_sub, cam_left, cam_right, toml_path='Follower_Consts.toml'
    ):
        self.vidcap_left = cv2.VideoCapture(cam_left)
        self.vidcap_right = cv2.VideoCapture(cam_right)
        self.const = toml.load(toml_path)["Lane_Follower"]
        # LOWER = np.array(self.const['lower_hsv'])
        # UPPER = np.array(self.const['upper_hsv'])
        # PTS1 = np.float32([tl, bl, tr, br])
        # PTS2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
        # # Matrix to warp the image for birdseye window
        # UNWARP = cv2.getPerspectiveTransform(pts1, pts2)
