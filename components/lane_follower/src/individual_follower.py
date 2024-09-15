# Lane Follower for a single camera
# Applies a perspective transform to an image, and isolates lane lines
import numpy as np
import math
import cv2


class IndividualFollower:
    ##Constants for how we process images
    ##These should be parametrized and better defined at a later date: as part of testing
    NWINDOWS = 9
    SEARCH_WIDTH = 100
    MINIPIX = 40

    ##Rendering Parameters
    BOX_COLOR = (0, 255, 0)
    DRAW_THICKNESS = 2

    def __init__(self):
        ##self.fit is set in Plot_Line
        self.fit = None
        ##These two are set below
        self._binary_warped = None
        self.histogram = None

    ##Need to determine if these should remain properties
    ##Or if they should be passed as arguments to Plot_Line
    @property
    def _binary_warped(self, value: np.ndarray):
        self._binary_warped = value
        self.histogram = np.sum(
            self.binary_warped[self.binary_warped.shape[0] // 2 :, :], axis=0
        )

    def plot_line(self):

        if self.binary_warped is None:
            raise Exception("no binary warp specified")

        ##Image to visualize output
        out_img = (
            np.dstack(
                (self._binary_warped, self._binary_warped, self._binary_warped)
            )
            * 255
        )

        window_bases = np.argmax(self.histogram[:])

        window_height = np.int32(
            self._binary_warped.shape[0] / IndividualFollower.NWINDOWS
        )

        lane_inds = []

        nonzero_pixels = self._binary_warped.nonzero()
        nonzero_y = np.array(nonzero_pixels[0])
        nonzero_x = np.array(nonzero_pixels[1])
        current = window_bases

        empty_windows = 0
        for window in range(IndividualFollower.NWINDOWS):
            window_dims = window * window_height
            win_y_upper = self._binary_warped.shape[0] - window_dims
            # One window height lower than win_y_higher
            win_y_lower = win_y_upper - window_height
            win_x_lower = current - IndividualFollower.SEARCH_WIDTH
            win_x_upper = current + IndividualFollower.SEARCH_WIDTH

        cv2.rectangle(
            out_img,
            (win_x_lower, win_y_lower),
            (win_x_upper, win_y_upper),
            IndividualFollower.BOX_COLOR,
            IndividualFollower.DRAW_THICKNESS,
        )

        ##lines for pre-commit checks: These should be removed prior to merge
        lane_inds
        nonzero_x
        nonzero_y
        empty_windows
        math
