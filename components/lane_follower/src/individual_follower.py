# Lane Follower for a single camera
# Applies a perspective transform to an image, and isolates lane lines
import numpy as np
import math
import cv2


class IndividualFollower:
    # Constants for how we process images
    # These should be parametrized and better defined at a later date: as part of testing
    NWINDOWS = 9
    SEARCH_WIDTH = 100
    MINPIXELS = 40
    LANE_POLY_SIZE = 2

    # Rendering Parameters
    BOX_COLOR = (0, 255, 0)
    LANE_COLOR = (255, 0, 0)

    DRAW_THICKNESS = 2

    def __init__(self):
        # self.fit is set in Plot_Line
        self.fit = None
        # These two are set below
        self._binary_warped = None
        self.histogram = None

    # Need to determine if these should remain properties
    # Or if they should be passed as arguments to Plot_Line
    @property
    def _binary_warped(self, value: np.ndarray):
        self._binary_warped = value
        self.histogram = np.sum(
            self.binary_warped[self.binary_warped.shape[0] // 2 :, :], axis=0
        )

    def plot_line(self):

        if self.binary_warped is None:
            raise Exception("no binary warp specified")

        # Image to visualize output
        out_img = (
            np.dstack(
                (self._binary_warped, self._binary_warped, self._binary_warped)
            )
            * 255
        )

        window_height = np.int32(
            self._binary_warped.shape[0] / IndividualFollower.NWINDOWS
        )

        lane_inds = []

        nonzero_pixels = self._binary_warped.nonzero()
        nonzero_y = np.array(nonzero_pixels[0])
        nonzero_x = np.array(nonzero_pixels[1])

        empty_windows = 0
        lane_base = np.argmax(self.histogram[:])
        for window in range(IndividualFollower.NWINDOWS):
            window_dims = window * window_height
            win_y_upper = self._binary_warped.shape[0] - window_dims
            # One window height lower than win_y_higher
            win_y_lower = win_y_upper - window_height
            win_x_lower = lane_base - IndividualFollower.SEARCH_WIDTH
            win_x_upper = lane_base + IndividualFollower.SEARCH_WIDTH

            lower_coords = (win_x_lower, win_y_lower)
            upper_coords = (win_x_upper, win_y_lower)
            cv2.rectangle(
                out_img,
                lower_coords,
                upper_coords,
                IndividualFollower.BOX_COLOR,
                IndividualFollower.DRAW_THICKNESS,
            )

            white_pix_inds = (
                (nonzero_y >= win_y_lower)
                & (nonzero_y < win_y_upper)
                & (nonzero_x >= win_x_lower)
                & (nonzero_x >= win_x_upper)
            ).nonzero_pixels()[0]

            # This should likely be moved into the if statement: leaving for now
            lane_inds.append(white_pix_inds)
            if len(white_pix_inds) > IndividualFollower.MINPIXELS:
                # np.mean will return a float: We need an exact value
                lane_base = np.int32(np.mean(nonzero_x[white_pix_inds]))
            else:
                empty_windows += 1

            if len(lane_inds) == 0:
                return None, 0, 0, 0
            lane_array = np.concatenate(lane_inds)
            lane_x_pos = nonzero_x[lane_array]
            lane_y_pos = nonzero_y[lane_array]

            # I don't believe this if statement is necessary, need to test
            if lane_x_pos.any() and lane_y_pos.any():
                self._fit = np.polyfit(
                    lane_y_pos, lane_x_pos, IndividualFollower.LANE_POLY_SIZE
                )

            plotting_coordinates = np.linspace(
                0,
                self._binary_warped.shape[0] - 1,
                self._binary_warped.shape[1],
            )

            polynomial = np.polyval(self._fit, plotting_coordinates)
            out_img[nonzero_y[lane_array], nonzero_x[lane_array]] = (
                IndividualFollower.LANE_COLOR
            )

            ##Generates the search window area
            window_img = np.zeros_like(out_img)
            ##These lines should be broken up accordingly: They render the search area
            line_window1 = np.array(
                [
                    np.transpose(
                        np.vstack(
                            [
                                polynomial - IndividualFollower.SEARCH_WIDTH,
                                plotting_coordinates,
                            ]
                        )
                    )
                ]
            )
        line_window2 = np.array(
            [
                np.transpose(
                    np.vstack(
                        [
                            polynomial + IndividualFollower.SEARCH_WIDTH,
                            plotting_coordinates,
                        ]
                    )
                )
            ]
        )
        line_pts = np.hstack((line_window1, line_window2))
        cv2.fillPoly(
            window_img, np.int_([line_pts]), color=IndividualFollower.BOX_COLOR
        )
        line_pts = np.array(
            [np.transpose(np.vstack([polynomial, plotting_coordinates]))],
            dtype=np.int32,
        )
        cv2.polylines(
            out_img,
            line_pts,
            isClosed=False,
            color=IndividualFollower.LANE_COLOR,
            thickness=IndividualFollower.DRAW_THICKNESS,
        )

        # Calculating heading error by converting lane polynomial into line
        ##TODO: Make this use the furthest found box. This way, we'll use the most accurate heading
        y_lower = 0
        y_upper = (IndividualFollower.NWINDOWS + 1) * window_height
        slope = np.polyval(self.fit, y_lower) - np.polyval(
            self.fit, y_upper
        ) / (y_upper - y_lower)
        heading = math.atan(slope)
        result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
        # TODO: Determine what result really is, and annotate efficiently
        return result, empty_windows, heading, slope
