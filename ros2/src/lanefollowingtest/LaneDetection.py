
# from irobot_create_msgs.msg import WheelTicks, WheelTicks
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np


# Inputs from both cameras
vidcap_left = cv2.VideoCapture("/dev/video0")
vidcap_right = cv2.VideoCapture("/dev/video2")


def nothing(x):
    pass


class Individual_Follower():
    def __init__(self):
        self._fit = None
        self._binary_warped = None

    def set_binwarp(self, binwarp):
        self._binary_warped = binwarp

    def Plot_Line(self, smoothen=False, prevFrameCount=6):
        histogram = np.sum(
            self._binary_warped[self._binary_warped.shape[0]//2:, :], axis=0)
        # Create an output image to draw on and  visualize the result
        out_img = np.dstack(
            (self._binary_warped, self._binary_warped, self._binary_warped))*255
        base = np.argmax(histogram[:])

        # THIS SHOULD BE SET CONSTANT AT INITIALIZATION
        nwindows = 9
        window_height = np.int32(self._binary_warped.shape[0]/nwindows)
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 20
        # Create empty list to hold pixel indices
        lane_inds = []

        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = self._binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        current = base

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = self._binary_warped.shape[0] - (window+1)*window_height
            win_y_high = self._binary_warped.shape[0] - window*window_height
            win_x_low = current - margin
            win_x_high = current + margin
            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high),
                          (0, 255, 0), 2)
            # Identify the nonzero pixels in x and y within the window
            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                         (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
            # Append these indices to the lists
            lane_inds.append(good_inds)
            if len(good_inds) > minpix:
                current = np.int32(np.mean(nonzerox[good_inds]))

        # Concatenate the arrays of indices
        if (lane_inds.size() > 0):
            lane_inds = np.concatenate(lane_inds)

        # Extract line pixel positions
        x_pos = nonzerox[lane_inds]
        y_pos = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        if (x_pos.any() and y_pos.any()):
            self._fit = np.polyfit(y_pos, x_pos, 2)
        else:
            return None

        # Ideas. Either use a from previous point thing
        # simple idea: just filter out the empty spots

        # TODO: Reimplement smoothen: talk to Isaiah if necessary or simply remmnant
        # if (smoothen):
        #     global fit_prev_left
        #     global fit_prev_right
        #     global fit_sum_left
        #     global fit_sum_right
        #     if (len(fit_prev_left) > prevFrameCount):
        #         fit_sum_left -= fit_prev_left.pop(0)
        #         fit_sum_right -= fit_prev_right.pop(0)

        #     fit_prev_left.append(self._left_fit)
        #     fit_prev_right.append(self._right_fit)
        #     fit_sum_left += self._left_fit
        #     fit_sum_right += self._right_fit

        #     no_of_fit_values = len(fit_prev_left)
        #     self._left_fit = fit_sum_left/no_of_fit_values
        #     self._right_fit = fit_sum_right/no_of_fit_values

        ploty = np.linspace(
            0, self._binary_warped.shape[0]-1, self._binary_warped.shape[0])
        fitx = self._fit[0]*ploty**2 + \
            self._fit[1]*ploty + self._fit[2]
        out_img[nonzeroy[lane_inds],
                nonzerox[lane_inds]] = [255, 0, 0]

        nonzero = self._binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        window_img = np.zeros_like(out_img)
        # Generate a polygon to illustrate the search window area
        # And recast the x and y points into usable format for cv2.fillPoly()
        line_window1 = np.array(
            [np.transpose(np.vstack([fitx-margin, ploty]))])
        line_window2 = np.array([np.flipud(np.transpose(np.vstack([fitx+margin,
                                                                   ploty])))])
        line_pts = np.hstack((line_window1, line_window2))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(window_img, np.int_([line_pts]), (0, 255, 0))
        line_pts = np.array(
            [np.transpose(np.vstack([fitx, ploty]))], dtype=np.int32)

        cv2.polylines(out_img, line_pts, isClosed=False,
                      color=(0, 255, 255), thickness=3)

        result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
        return result


class Lane_Follower(Node):

    def __init__(self):
        super().__init__('lane_detection_node')
        self._left_follower = Individual_Follower()
        self._right_follower = Individual_Follower()

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Publisher for error from the lane lines relative to the camera FOV
        self.camData_publisher = self.create_publisher(
            Float64, '/cam_data', 10)

    def measure_position_meters(self):
        left_fit = self._left_follower._fit
        right_fit = self._right_follower._fit

        # Will be the same for left side & right side
        y_max = self._left_follower._binary_warped.shape[0]

        # Define conversion in x from pixels space to meters
        # center is roughly 0.005

        xm_per_pix = 1  # meters per pixel in x dimension
        # Choose the y value corresponding to the bottom of the image
        # Calculate left and right line positions at the bottom of the image
        left_x_pos = left_fit[0]*y_max**2 + \
            left_fit[1]*y_max + left_fit[2]

        right_x_pos = right_fit[0]*y_max**2 + \
            right_fit[1]*y_max + right_fit[2]

        # Alternative method
        center_lane_offset = (right_x_pos - left_x_pos)

        # TODO: This center lane math might actually translate over 1-1: Will need to discuss & test
        # Calculate the x position of the center of the lane
        center_lanes_x_pos = (left_x_pos + right_x_pos)//2
        # Calculate the deviation between the center of the lane and the center of the picture
        # The car is assumed to be placed in the center of the picture
        # If the deviation is negative, the car is on the felt hand side of the center of the lane
        veh_pos = (
            (self._binary_warped.shape[1]//2) - center_lanes_x_pos) * xm_per_pix
        return veh_pos / 100

    def timer_callback(self):
        # TODO: Move these out into a separate calibrator class that saves values  to a .csv
        # THESE SHOULD NOT BE GENERATE ON EVERY TIMER CALLBACK, BAD DESIGN (ON MY PART)
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L - V", "Trackbars", 200, 255, nothing)
        cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("U - S", "Trackbars", 50, 255, nothing)
        cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
        lower = np.array([l_h, l_s, l_v])
        upper = np.array([u_h, u_s, u_v])
        # ## Choosing points for perspective transformation
        # THESE ARE PRESET, MODIFYING
        # tl = (222,387)
        # bl = (70 ,472)
        # tr = (400,380)
        # br = (538,472)
        bl = (12, 355)
        tl = (66, 304)
        br = (635, 344)
        tr = (595, 308)
        # Aplying perspective transformation
        pts1 = np.float32([tl, bl, tr, br])
        pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
        # Matrix to warp the image for birdseye window
        matrix = cv2.getPerspectiveTransform(pts1, pts2)

        ## ALL OF THIS SHOULD BE PROCESSED ONLY ONCE 

        success_l, image_l = vidcap_left.read()
        success_r, image_r = vidcap_right.read()
        images = [(image_l, "Left"), (image_r, "Right")]
        if not(success_l and success_r):
            return        
        for image in images:
            frame = cv2.resize(image[0], (640, 480))
            # I might've cooked with this list comprehension
            (cv2.circle(frame,point,5,(0,0,255),-1) for point in (bl,tl,br,tr))
            transformed_frame = cv2.warpPerspective(
                frame, matrix, (640, 480))
            # Object Detection
            # Image Thresholding
            hsv_transformed_frame = cv2.cvtColor(
                transformed_frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_transformed_frame, lower, upper)
            if image[1] == "Left":
                self._left_follower.set_binwarp(binwarp=mask)
            else:
                self._right_follower.set_binwarp(binwarp=mask)

        result_left = self._left_follower.Plot_Line()
        result_right = self._right_follower.Plot_Line()
        if (result_left is not None and result_right is not None):
            pos = self.measure_position_meters()
            print(pos)
            msg_out = Float64()
            msg_out.data = pos
            self.camData_publisher.publish(msg_out)
            cv2.imshow("Result Left", result_left)
            cv2.imshow("Result Right", result_right)
            msg_out = Float64()
            msg_out.data = pos
            self.camData_publisher.publish(msg_out)  # Publish the error
        cv2.imshow("Original", frame)
        cv2.imshow("Bird's Eye View", transformed_frame)
        if cv2.waitKey(10) == 27:
            return



            

            # # TODO: Fix renderer
            #     # Histogram
            # histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
            # midpoint = np.int32(histogram.shape[0]/2)
            # left_base = np.argmax(histogram[:midpoint])
            # right_base = np.argmax(histogram[midpoint:]) + midpoint

            # # Sliding Window
            # y = 472
            # lx = []
            # rx = []

            # # Set the width of the windows +/- margin
            # margin = 100
            # # Set minimum number of pixels found to recenter window
            # minpix = 50

            # msk = mask.copy()
            # output = self.Plot_line()
            # if (output is not None):
            #     out_img, result, left_fitx, right_fitx, \
            #         ploty, left_fit, right_fit, left_lane_inds, right_lane_inds, lane_width = output
            #     pos = self.measure_position_meters()
            #     print(pos)

            #     msg_out = Float64()
            #     msg_out.data = pos
            #     self.camData_publisher.publish(msg_out)  # Publish the error

            #     while y > 0:
            #         nonzero = msk.nonzero()
            #         nonzeroy = np.array(nonzero[0])
            #         nonzerox = np.array(nonzero[1])
            #         # print(nonzero)

            #         # Left threshold
            #         img = mask[y-40:y, left_base-50:left_base+50]
            #         contours, _ = cv2.findContours(
            #             img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            #         for contour in contours:
            #             M = cv2.moments(contour)
            #             if M["m00"] != 0:
            #                 cx = int(M["m10"]/M["m00"])
            #                 cy = int(M["m01"]/M["m00"])
            #                 lx.append(left_base-50 + cx)
            #                 left_base = left_base-50 + cx

            #         # Right threshold
            #         img = mask[y-40:y, right_base-50:right_base+50]
            #         contours, _ = cv2.findContours(
            #             img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            #         for contour in contours:
            #             M = cv2.moments(contour)
            #             if M["m00"] != 0:
            #                 cx = int(M["m10"]/M["m00"])
            #                 cy = int(M["m01"]/M["m00"])
            #                 rx.append(right_base-50 + cx)
            #                 right_base = right_base-50 + cx

            #         cv2.rectangle(msk, (left_base-50, y),
            #                       (left_base+50, y-40), (255, 255, 255), 2)
            #         cv2.rectangle(msk, (right_base-50, y),
            #                       (right_base+50, y-40), (255, 255, 255), 2)
            #         y -= 40
            #         try:
            #             leftx = nonzerox[lx]
            #             lefty = nonzeroy[lx]
            #             rightx = nonzerox[rx]
            #             righty = nonzeroy[rx]
            #         except:
            #             print("error this time around, retrying")
            #         # cv2.imshow("Outimg" , out_img)
            #         cv2.imshow("R", result)

            # # TODO: Fix rendering for these
            # cv2.imshow("Lane Detection - Image Thresholding", mask)
            # cv2.imshow("Lane Detection - Sliding Windows", msk)
            # Display the result
                # if bool(lx) and bool(rx):
                #     left_fit = np.polyfit(lefty, leftx, 2)
                #     right_fit = np.polyfit(righty, rightx, 2)

                #         # Generate y values for the entire height of the image
                #     ploty = np.linspace(0, transformed_frame.shape[0] - 1, transformed_frame.shape[0])

                #     # Generate x values using the polynomial fits
                #     left_fitx = np.polyval(left_fit, ploty)
                #     right_fitx = np.polyval(right_fit, ploty)

                #     # Create an image to draw the lane lines
                #     line_image = np.zeros_like(msk)

                #     # Draw the left lane line
                #     for i in range(len(left_fitx)):
                #         cv2.circle(line_image, (int(left_fitx[i]), int(ploty[i])), 1, 255, -1)

                #     # Draw the right lane line
                #     for i in range(len(right_fitx)):
                #         cv2.circle(line_image, (int(right_fitx[i]), int(ploty[i])), 1, 255, -1)

                #     # Combine the original image with the drawn lane lines
                #     result = cv2.addWeighted(mask, 1, cv2.cvtColor(line_image, cv2.COLOR_GRAY2BGR), 0.3, 0)



def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 program
    node = Lane_Follower()  # Instantiate Node
    rclpy.spin(node)  # Puts the node in an infinite loop
    # Clean shutdown should be at the end of every node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
