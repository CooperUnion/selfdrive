import math
import numpy as np
import cv2


class Individual_Follower:
    def __init__(self):

        self._fit = None
        self._binary_warped = None

    def set_binwarp(self, binwarp: np.ndarray):
        self._binary_warped = binwarp

    def Plot_Line(self, smoothen=False, prevFrameCount=6):
        # Number of windows for sliding windows
        nwindows = 9
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 20

        histogram = np.sum(
            self._binary_warped[self._binary_warped.shape[0] // 2 :, :], axis=0
        )
        # Create an output image to draw on and visualize the result
        out_img = (
            np.dstack(
                (self._binary_warped, self._binary_warped, self._binary_warped)
            )
            * 255
        )
        base = np.argmax(histogram[:])

        window_height = np.int32(self._binary_warped.shape[0] / nwindows)
        # Create empty list to hold pixel indices
        lane_inds = []

        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = self._binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        current = base

        # Used for calculating the number of "empty windows".
        # Relevant for our lane determination: the more empty windows, the more likely this is the dashed line
        empty_windows = 0
        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = (
                self._binary_warped.shape[0] - (window + 1) * window_height
            )
            win_y_high = self._binary_warped.shape[0] - window * window_height
            win_x_low = current - margin
            win_x_high = current + margin
            # Draw the windows on the visualization image
            cv2.rectangle(
                out_img,
                (win_x_low, win_y_low),
                (win_x_high, win_y_high),
                (0, 255, 0),
                2,
            )
            # Identify the nonzero pixels in x and y within the window: inclusive of lower bounds
            good_inds = (
                (nonzeroy >= win_y_low)
                & (nonzeroy < win_y_high)
                & (nonzerox >= win_x_low)
                & (nonzerox < win_x_high)
            ).nonzero()[0]

            # Append these indices to the lists
            lane_inds.append(good_inds)
            if len(good_inds) > minpix:
                current = np.int32(np.mean(nonzerox[good_inds]))
            else:
                empty_windows += 1

        # Concatenate the arrays of indices
        if len(lane_inds) > 0:
            lane_inds = np.concatenate(lane_inds)

        # Extract line pixel positions
        x_pos = nonzerox[lane_inds]
        y_pos = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        if x_pos.any() and y_pos.any():
            self._fit = np.polyfit(y_pos, x_pos, 2)
        else:
            return None, empty_windows, 0

        ploty = np.linspace(
            0, self._binary_warped.shape[0] - 1, self._binary_warped.shape[0]
        )
        fitx = np.polyval(self._fit, ploty)
        out_img[nonzeroy[lane_inds], nonzerox[lane_inds]] = [255, 0, 0]

        nonzero = self._binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        window_img = np.zeros_like(out_img)
        # Generate a polygon to illustrate the search window area
        # And recast the x and y points into usable format for cv2.fillPoly()
        line_window1 = np.array(
            [np.transpose(np.vstack([fitx - margin, ploty]))]
        )
        line_window2 = np.array(
            [(np.transpose(np.vstack([fitx + margin, ploty])))]
        )
        line_pts = np.hstack((line_window1, line_window2))

        # Draw the lane onto the warped blank imleft_bufferage
        cv2.fillPoly(window_img, np.int_([line_pts]), color=(0, 255, 0))
        line_pts = np.array(
            [np.transpose(np.vstack([fitx, ploty]))], dtype=np.int32
        )

        cv2.polylines(
            out_img, line_pts, isClosed=False, color=(0, 255, 255), thickness=3
        )
        # Evaluating heading error:

        # This is the first window coordinates
        y1 = self._binary_warped.shape[0] - (nwindows) * window_height
        # This is the second window coordinates
        y2 = self._binary_warped.shape[0] - (nwindows + 1) * window_height
        # Returns polynomial values at a point
        x1 = np.polyval(self._fit, y1)
        x2 = np.polyval(self._fit, y2)

        slope = (x2 - x1) / (y2 - y1)

        heading = math.atan((x2 - x1) / (y2 - y1))  # Radians
        result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
        return result, empty_windows, heading, slope
