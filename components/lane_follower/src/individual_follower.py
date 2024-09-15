# Lane Follower for a single camera
# Applies a perspective transform to an image, and isolates lane lines
import numpy as np
import math
import cv2


class Individual_Follower:
    NWINDOWS = 9
    MARGIN = 100
    MINIPIX = 40

    def __init__(self, fit, binary_warped):
        self.fit = fit
        self._binary_warped = binary_warped
        self.histogram = np.sum(
            binary_warped[binary_warped.shape[0] // 2 :, :], axis=0
        )

    @property
    def _binary_warped(self, value):
        self._binary_warped = value
        self.histogram = np.sum(
            self.binary_warped[self.binary_warped.shape[0] // 2 :, :], axis=0
        )

    def Plot_Line(self):
        out_img = (
            np.dstack(
                (self._binary_warped, self._binary_warped, self._binary_warped)
            )
            * 255
        )
        base = np.argmax(self.histogram[:])
        ##lines for pre-commit checks: These should be removed prior to merge
        math
        cv2
        out_img
        base
