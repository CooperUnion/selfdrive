import math

import cv2
import numpy as np
import toml
from einops import repeat


class ifResult:
    def __init__(self):
        self.result_img = None
        self.empty_windows = 0
        self.heading = 0
        self.slope = 0

    pass


class IndividualFollower:

    def __init__(self, toml_path='Follower_Consts.toml'):

        self.consts = toml.load(toml_path)["Individual_Follower"]
        # self.fit is set in Plot_Line
        self.fit = None
        # These two are set below
        self._binary_warped = None
        self._histogram = None

    # Need to determine if these should remain properties
    # Or if they should be passed as arguments to Plot_Line
    @property
    def _binary_warped(self, value: np.ndarray):
        self._binary_warped = value
        self._histogram = np.sum(
            self.binary_warped[self.binary_warped.shape[0] // 2 :, :], axis=0
        )

    def get_white_pixels(self, window, window_height, out_img=None):
        NWINDOWS = self.consts["NWINDOWS"]
        SEARCH_WIDTH = self.consts['SEARCH_WIDTH']
        lane_inds = []

        nonzero_pixels = self._binary_warped.nonzero()
        nonzero_y = np.array(nonzero_pixels[0])
        nonzero_x = np.array(nonzero_pixels[1])

        empty_windows = 0
        lane_base = np.argmax(self.histogram[:])

        for window in range(NWINDOWS):
            window_dims = window * window_height
            win_y_upper = self._binary_warped.shape[0] - window_dims
            # One window height lower than win_y_higher
            win_y_lower = win_y_upper - window_height
            win_x_lower = lane_base - SEARCH_WIDTH
            win_x_upper = lane_base + SEARCH_WIDTH

            lower_coords = (win_x_lower, win_y_lower)
            upper_coords = (win_x_upper, win_y_lower)
            cv2.rectangle(
                out_img,
                lower_coords,
                upper_coords,
                self.consts["BOX_COLOR"],
                self.consts["DRAW_THICKNESS"],
            )

            white_pix_inds = (
                (nonzero_y >= win_y_lower)
                & (nonzero_y < win_y_upper)
                & (nonzero_x >= win_x_lower)
                & (nonzero_x >= win_x_upper)
            ).nonzero_pixels()[0]

            # This should likely be moved into the if statement: leaving for now
            lane_inds.append(white_pix_inds)
            if len(white_pix_inds) > self.consts["MINPIXELS"]:
                # np.mean will return a float: We need an exact value
                lane_base = np.int32(np.mean(nonzero_x[white_pix_inds]))
            else:
                empty_windows += 1

            # if len(lane_inds) == 0:
            #     return result

            lane_array = np.concatenate(lane_inds)
            lane_x_pos = nonzero_x[lane_array]
            lane_y_pos = nonzero_y[lane_array]
            # TODO: test if this statement is necessary
            if lane_x_pos.any() and lane_y_pos.any():
                self._fit = np.polyfit(
                    lane_y_pos, lane_x_pos, self.consts["LANE_POLY_SIZE"]
                )

            out_img[nonzero_y[lane_array], nonzero_x[lane_array]] = (
                self.consts["LANE_COLOR"]
            )

    def plot_line(self) -> ifResult:
        if not self._binary_warped:
            raise Exception("no binary warp specified")

        # Image to visualize output
        out_img = repeat(self._binary_warped, 'h w -> h w c', repeat=3) * 255
        ##These outputs need to be confirmed compatible
        # out_img = (
        #     np.dstack(
        #         self._binary_warped, self._binary_warped, self._binary_warped
        #     )
        #     * 255
        # )

        NWINDOWS = self.consts['NWINDOWS']

        window_height = np.int32(self._binary_warped.shape[0] / NWINDOWS)
        ##Create result class:
        result = ifResult()

        lane_inds = []

        nonzero_pixels = self._binary_warped.nonzero()
        nonzero_y = np.array(nonzero_pixels[0])
        nonzero_x = np.array(nonzero_pixels[1])

        empty_windows = 0
        lane_base = np.argmax(self.histogram[:])

        SEARCH_WIDTH = self.consts['SEARCH_WIDTH']
        for window in range(NWINDOWS):
            window_dims = window * window_height
            win_y_upper = self._binary_warped.shape[0] - window_dims
            # One window height lower than win_y_higher
            win_y_lower = win_y_upper - window_height
            win_x_lower = lane_base - SEARCH_WIDTH
            win_x_upper = lane_base + SEARCH_WIDTH

            lower_coords = (win_x_lower, win_y_lower)
            upper_coords = (win_x_upper, win_y_lower)
            cv2.rectangle(
                out_img,
                lower_coords,
                upper_coords,
                self.consts["BOX_COLOR"],
                self.consts["DRAW_THICKNESS"],
            )

            white_pix_inds = (
                (nonzero_y >= win_y_lower)
                & (nonzero_y < win_y_upper)
                & (nonzero_x >= win_x_lower)
                & (nonzero_x >= win_x_upper)
            ).nonzero_pixels()[0]

            # This should likely be moved into the if statement: leaving for now
            lane_inds.append(white_pix_inds)
            if len(white_pix_inds) > self.consts["MINPIXELS"]:
                # np.mean will return a float: We need an exact value
                lane_base = np.int32(np.mean(nonzero_x[white_pix_inds]))
            else:
                empty_windows += 1

            if len(lane_inds) == 0:
                return result

            lane_array = np.concatenate(lane_inds)
            lane_x_pos = nonzero_x[lane_array]
            lane_y_pos = nonzero_y[lane_array]
            # TODO: test if this statement is necessary
            if lane_x_pos.any() and lane_y_pos.any():
                self._fit = np.polyfit(
                    lane_y_pos, lane_x_pos, self.consts["LANE_POLY_SIZE"]
                )

            out_img[nonzero_y[lane_array], nonzero_x[lane_array]] = (
                self.consts["LANE_COLOR"]
            )

        ##Generates the search window area
        window_img = np.zeros_like(out_img)

        # Create linearly spaced points at each height, evaluate polynomial, create coordinates
        x_val = np.arange(0, window_height, step=5).T
        y_val = np.polyval(self.fit, x_val)
        coords = np.concatenate((x_val, y_val), axis=0)

        ##TODO: Test if this approach works

        cv2.polylines(
            out_img,
            coords,
            isClosed=False,
            color=self.consts["LANE_COLOR"],
            thickness=self.consts["DRAW_THICKNESS"],
        )

        # Calculating heading error by converting lane polynomial into line
        ##TODO: Make this use the furthest found box. This way, we'll use the most accurate heading
        y_lower = 0
        y_upper = (NWINDOWS + 1) * window_height

        result.slope = np.polyval(self.fit, y_lower) - np.polyval(
            self.fit, y_upper
        ) / (y_upper - y_lower)
        result.result_img = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
        result.heading = math.atan(result.slope)

        return result
