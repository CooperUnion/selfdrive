
# from irobot_create_msgs.msg import WheelTicks, WheelTicks 
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float64

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
import numpy as np

vidcap = cv2.VideoCapture("/dev/video0")

def nothing(x):
    pass

class Lane_Follower(Node):
    
    def __init__(self):
        super().__init__('lane_detection_node')

        self._left_fit = None
        self._right_fit = None
        self._binary_warped = None

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.camData_publisher = self.create_publisher(Float64, '/cam_data', 10) # Publisher for error from the lane lines relative to the camera FOV


    def set_binwarp(self, binwarp):
        self._binary_warped = binwarp

    def Plot_line(self, smoothen=False,prevFrameCount=6): #used Udacity's code to plot the lines and windows over lanes 
        histogram = np.sum(self._binary_warped[self._binary_warped.shape[0]//2:, :], axis=0)
        # histogram = np.sum(self._binary_warped[self._binary_warped.shape[0]//2:,:], axis=0)
        # Create an output image to draw on and  visualize the result
        out_img = np.dstack((self._binary_warped, self._binary_warped, self._binary_warped))*255
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int32(histogram.shape[0]/2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        lane_width= abs(rightx_base-leftx_base)
        # Choose the number of sliding windows
        nwindows = 9
        # Set height of windows
        window_height = np.int32(self._binary_warped.shape[0]/nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = self._binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated for each window
        leftx_current = leftx_base
        rightx_current = rightx_base
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 20
        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = self._binary_warped.shape[0] - (window+1)*window_height
            win_y_high = self._binary_warped.shape[0] - window*window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
            (0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
            (0,255,0), 2) 
            # Identify the nonzero pixels in x and y within the window
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:        
                rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            pass


        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds] 


        # Fit a second order polynomial to each
        if(lefty.any() and leftx.any()):
            self._left_fit = np.polyfit(lefty, leftx, 2)
        else: 
            return None
        if(righty.any() and rightx.any()):
            self._right_fit = np.polyfit(righty, rightx, 2)
        else:
            return None

        # Ideas. Either use a from pervious point thing
        # simple idea: just filter out the empty spots 
        
        if(smoothen):
            global fit_prev_left
            global fit_prev_right
            global fit_sum_left
            global fit_sum_right
            if(len(fit_prev_left)>prevFrameCount):
                fit_sum_left-= fit_prev_left.pop(0)
                fit_sum_right-= fit_prev_right.pop(0)

            fit_prev_left.append(self._left_fit)
            fit_prev_right.append(self._right_fit)
            fit_sum_left+=self._left_fit
            fit_sum_right+= self._right_fit

            no_of_fit_values=len(fit_prev_left) 
            self._left_fit= fit_sum_left/no_of_fit_values
            self._right_fit= fit_sum_right/no_of_fit_values
        
        
        ploty = np.linspace(0, self._binary_warped.shape[0]-1, self._binary_warped.shape[0] )
        left_fitx = self._left_fit[0]*ploty**2 + self._left_fit[1]*ploty + self._left_fit[2]
        right_fitx = self._right_fit[0]*ploty**2 + self._right_fit[1]*ploty + self._right_fit[2]

        out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
        out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
        
        nonzero = self._binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        window_img = np.zeros_like(out_img)
        # Generate a polygon to illustrate the search window area
        # And recast the x and y points into usable format for cv2.fillPoly()
        left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
        left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin, 
                                    ploty])))])
        left_line_pts = np.hstack((left_line_window1, left_line_window2))
        right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
        right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, 
                                    ploty])))])
        right_line_pts = np.hstack((right_line_window1, right_line_window2))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
        cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))

        left_line_pts = np.array([np.transpose(np.vstack([left_fitx, ploty]))], dtype=np.int32)
        right_line_pts = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))], dtype=np.int32)

        cv2.polylines(out_img, left_line_pts, isClosed=False, color=(0, 255, 255), thickness=3)
        cv2.polylines(out_img, right_line_pts, isClosed=False, color=(0, 255, 255), thickness=3)

        result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

        return out_img, result, left_fitx,right_fitx,ploty,self._left_fit, self._right_fit, \
        left_lane_inds,right_lane_inds,lane_width

    def measure_position_meters(self):
        # Define conversion in x from pixels space to meters

        # center is roughly 0.005
        xm_per_pix = 1 # meters per pixel in x dimension
        # Choose the y value corresponding to the bottom of the image
        y_max = self._binary_warped.shape[0]
        # Calculate left and right line positions at the bottom of the image
        left_x_pos = self._left_fit[0]*y_max**2 + self._left_fit[1]*y_max + self._left_fit[2]
        right_x_pos = self._right_fit[0]*y_max**2 + self._right_fit[1]*y_max + self._right_fit[2] 
        # Calculate the x position of the center of the lane 
        center_lanes_x_pos = (left_x_pos + right_x_pos)//2
        # Calculate the deviation between the center of the lane and the center of the picture
        # The car is assumed to be placed in the center of the picture
        # If the deviation is negative, the car is on the felt hand side of the center of the lane
        veh_pos = ((self._binary_warped.shape[1]//2) - center_lanes_x_pos) * xm_per_pix
        return veh_pos / 100
        

    
    def timer_callback(self):
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L - V", "Trackbars", 200, 255, nothing)
        cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("U - S", "Trackbars", 50, 255, nothing)
        cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

        success, image = vidcap.read()
        if success:
            frame = cv2.resize(image, (640,480))

            # ## Choosing points for perspective transformation
            # THESE ARE PRESET, MODIFYING
            # tl = (222,387)
            # bl = (70 ,472)
            # tr = (400,380)
            # br = (538,472)
            bl = (12,355)
            tl = (66,304)
            br = (635,344)
            tr = (595,308)


            cv2.circle(frame, tl, 5, (0,0,255), -1)
            cv2.circle(frame, bl, 5, (0,0,255), -1)
            cv2.circle(frame, tr, 5, (0,0,255), -1)
            cv2.circle(frame, br, 5, (0,0,255), -1)

            ## Aplying perspective transformation
            pts1 = np.float32([tl, bl, tr, br]) 
            pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]]) 
            
            # Matrix to warp the image for birdseye window
            matrix = cv2.getPerspectiveTransform(pts1, pts2) 
            transformed_frame = cv2.warpPerspective(frame, matrix, (640,480))

            ### Object Detection
            # Image Thresholding
            hsv_transformed_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)
            
            l_h = cv2.getTrackbarPos("L - H", "Trackbars")
            l_s = cv2.getTrackbarPos("L - S", "Trackbars")
            l_v = cv2.getTrackbarPos("L - V", "Trackbars")
            u_h = cv2.getTrackbarPos("U - H", "Trackbars")
            u_s = cv2.getTrackbarPos("U - S", "Trackbars")
            u_v = cv2.getTrackbarPos("U - V", "Trackbars")
            
            lower = np.array([l_h,l_s,l_v])
            upper = np.array([u_h,u_s,u_v])
            mask = cv2.inRange(hsv_transformed_frame, lower, upper)

            self.set_binwarp(binwarp=mask) 


            #Histogram
            histogram = np.sum(mask[mask.shape[0]//2:, :], axis=0)
            midpoint = np.int32(histogram.shape[0]/2)
            left_base = np.argmax(histogram[:midpoint])
            right_base = np.argmax(histogram[midpoint:]) + midpoint

            #Sliding Window
            y = 472
            lx = []
            rx = []

            # Set the width of the windows +/- margin
            margin = 100
            # Set minimum number of pixels found to recenter window
            minpix = 50

            msk = mask.copy()
            output = self.Plot_line()
            if(output is not None):
                out_img, result, left_fitx,right_fitx, \
                ploty,left_fit, right_fit, left_lane_inds,right_lane_inds,lane_width = output
                pos = self.measure_position_meters()
                print(pos)

                msg_out = Float64()
                msg_out.data = pos
                self.camData_publisher.publish(msg_out) # Publish the error 
                
                while y>0:
                    nonzero = msk.nonzero()
                    nonzeroy = np.array(nonzero[0])
                    nonzerox = np.array(nonzero[1])
                    # print(nonzero)


                    ## Left threshold
                    img = mask[y-40:y, left_base-50:left_base+50]
                    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    for contour in contours:
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"]/M["m00"])
                            cy = int(M["m01"]/M["m00"])
                            lx.append(left_base-50 + cx)
                            left_base = left_base-50 + cx
                    
                    ## Right threshold
                    img = mask[y-40:y, right_base-50:right_base+50]
                    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    for contour in contours:
                        M = cv2.moments(contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"]/M["m00"])
                            cy = int(M["m01"]/M["m00"])
                            rx.append(right_base-50 + cx)
                            right_base = right_base-50 + cx


                    
                    cv2.rectangle(msk, (left_base-50,y), (left_base+50,y-40), (255,255,255), 2)
                    cv2.rectangle(msk, (right_base-50,y), (right_base+50,y-40), (255,255,255), 2)
                    y -= 40
                    try:
                        leftx = nonzerox[lx]
                        lefty = nonzeroy[lx] 
                        rightx = nonzerox[rx]
                        righty = nonzeroy[rx]                
                    except:
                        print("error this time around, retrying")
                    # cv2.imshow("Outimg" , out_img)
                    cv2.imshow("R",result)

            cv2.imshow("Original", frame)
            cv2.imshow("Bird's Eye View", transformed_frame)
            cv2.imshow("Lane Detection - Image Thresholding", mask)
            cv2.imshow("Lane Detection - Sliding Windows", msk)
                    # Display the result
            if cv2.waitKey(10) == 27:
                pass               
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
    rclpy.init(args=args) # Initialize ROS2 program
    node = Lane_Follower() # Instantiate Node
    rclpy.spin(node) # Puts the node in an infinite loop
    
    # Clean shutdown should be at the end of every node
    node.destroy_node() 
    rclpy.shutdown()


if __name__ == '__main__':
    main()