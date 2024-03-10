import os
import glob

import cv2
import numpy as np
import matplotlib.pyplot as plt
from moviepy.editor import VideoFileClip
from whitepxlClass import WhitePixelDetector


GRADIENT_THRESH = (20, 100)
S_CHANNEL_THRESH = (80, 255)
L_CHANNEL_THRESH = (80, 255)

# NEED TO BE
B_CHANNEL_THRESH = (150, 200)
L2_CHANNEL_THRESH = (200, 200)


class CameraCalibration:
    def __init__(self, nx, ny, chessboard_dir,SHOW_PLOT=False):
        self.NX = nx
        self.NY = ny
        self.CHESSBOARD_DIR = chessboard_dir
        self.show_plot = SHOW_PLOT

    def get_chessboard_corners(self, imgs):
        """Return image and object points from a set of chessboard images."""
        if not isinstance(imgs, list):
            raise ValueError("imgs parameter needs to be a list.")
        
        # Initialize 3D object points
        objp = np.zeros((self.NX * self.NY, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.NX, 0:self.NY].T.reshape(-1, 2)
        
        imgps = []
        objps = []
        
        for img in imgs:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, (self.NX, self.NY), None)
            if not found:
                continue
            imgps.append(corners)
            objps.append(objp)
        
        return imgps, objps

    def chessboard_cam_calib(self, imgps, objps, img_size):
        """Returns camera calibration matrix and distortion coefficients."""
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objps, imgps, img_size, None, None)
        return mtx, dist

    def plots_and_calibration(self):
        img_locs = glob.glob(self.CHESSBOARD_DIR)
        print(img_locs)

        # Read all images into a list
        imgs = [cv2.imread(img_loc) for img_loc in img_locs]
        


        # Get size of images using one image as a sample, with width as the first element
        img_size = imgs[0].shape[::-1][1:]

        # Calibrate camera and retrieve calibration matrix and distortion coefficients
        imgps, objps = self.get_chessboard_corners(imgs)
        MTX, DIST = self.chessboard_cam_calib(imgps, objps, img_size)

        if self.show_plot:

            # Set up figure for plotting
            f, axarr = plt.subplots(len(imgs), 3)
            f.set_size_inches(15, 30)

            # Loop through images, undistort them, and draw corners on undistorted versions.
            for i, img in enumerate(imgs):
                # Set column headings on figure
                if i == 0:
                    axarr[i, 0].set_title("Original Image")
                    axarr[i, 1].set_title("Undistorted Image")
                    axarr[i, 2].set_title("Corners Drawn")

                # Generate new undistorted image
                undist = cv2.undistort(img, MTX, DIST, None, MTX)
                undist_copy = undist.copy()

                # Generate new image with corner points drawn
                undist_grey = cv2.cvtColor(undist_copy, cv2.COLOR_BGR2GRAY)
                found, corners = cv2.findChessboardCorners(undist_grey, (self.NX, self.NY), None)

                if found:
                    drawn = cv2.drawChessboardCorners(undist_copy, (self.NX, self.NY), corners, found)
                else:
                    drawn = img

                # Plot images on figure
                axarr[i, 0].imshow(img)
                axarr[i, 0].axis('off')
                axarr[i, 1].imshow(undist)
                axarr[i, 1].axis('off')
                axarr[i, 2].imshow(drawn)
                axarr[i, 2].axis('off')
        return MTX,DIST
    

def seperate_hls(rgb_img):
    hls = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HLS)
    h = hls[:,:,0]
    l = hls[:,:,1]
    s = hls[:,:,2]
    return h, l, s

def seperate_lab(rgb_img):
    lab = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2Lab)
    l = lab[:,:,0]
    a = lab[:,:,1]
    b = lab[:,:,2]
    return l, a, b

def seperate_luv(rgb_img):
    luv = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2Luv)

    l = luv[:,:,0]
    u = luv[:,:,1]
    v = luv[:,:,2]
    return l, u, v

def binary_threshold_lab_luv(rgb_img, bthresh, lthresh):
    l, a, b = seperate_lab(rgb_img)
    l2, u, v = seperate_luv(rgb_img)
    binary = np.zeros_like(l)
    binary[
        ((b > bthresh[0]) & (b <= bthresh[1])) |
        ((l2 > lthresh[0]) & (l2 <= lthresh[1]))
    ] = 1
    return binary

def binary_threshold_hls(rgb_img, sthresh, lthresh):
    h, l, s = seperate_hls(rgb_img)
    binary = np.zeros_like(h)
    binary[
        ((s > sthresh[0]) & (s <= sthresh[1])) &
        ((l > lthresh[0]) & (l <= lthresh[1]))
    ] = 1
    return binary

def gradient_threshold(channel, thresh):
    # Take the derivative in x
    sobelx = cv2.Sobel(channel, cv2.CV_64F, 1, 0)
    # Absolute x derivative to accentuate lines away from horizontal
    abs_sobelx = np.absolute(sobelx)
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    # Threshold gradient channel
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
    return sxbinary


def histo_peak(histo):
    """Find left and right peaks of histogram"""
    midpoint = np.int32(histo.shape[0]/2)
    leftx_base = np.argmax(histo[:midpoint])
    rightx_base = np.argmax(histo[midpoint:]) + midpoint
    return leftx_base, rightx_base

def get_lane_indices_sliding_windows(binary_warped, leftx_base, rightx_base, n_windows, margin, recenter_minpix):
    """Get lane line pixel indices by using sliding window technique"""
    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    out_img = out_img.copy()
    # Set height of windows
    window_height = np.int32(binary_warped.shape[0]/n_windows)
    
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    
    for window in range(n_windows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high), (0,255,0), 2)
        cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high), (0,255,0), 2)
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                          (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                           (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > recenter_minpix:
            leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > recenter_minpix:        
            rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))
        
    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)
    return left_lane_inds, right_lane_inds, nonzerox, nonzeroy, out_img

def get_lane_indices_from_prev_window(binary_warped, left_fit, right_fit, margin):
    """Detect lane line by searching around detection of previous sliding window detection"""
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + 
    left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) + 
    left_fit[1]*nonzeroy + left_fit[2] + margin))) 
    right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + 
    right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) + 
    right_fit[1]*nonzeroy + right_fit[2] + margin)))
    
    # Again, extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    
    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    
    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    return left_lane_inds, right_lane_inds, ploty, left_fitx, right_fitx





            
def main():
    # Set parameters
    cap = cv2.VideoCapture("/dev/video2")
    
    
    nx = 9
    ny = 6
    chessboard_dir = './camera_cal_adesso/*.jpg'

    # Create an instance of the CameraCalibration class
    calibrator = CameraCalibration(nx, ny, chessboard_dir)
    

    # Run the calibration and plotting
    MTX,DIST = calibrator.plots_and_calibration()


    ret = True
    while ret:
        ret, frame = cap.read()

        # cv2.imshow("INITAL FRAME", frame)


        # TEST_IMG = "./test_images/straight_lines1.jpg"
        lane_test_img = frame
        lane_test_img_rgb = cv2.cvtColor(lane_test_img, cv2.COLOR_BGR2RGB)
        lane_test_undist = cv2.undistort(lane_test_img_rgb, MTX, DIST, None, MTX)
        reg_distort = np.hstack((frame, lane_test_undist))

        # Display the combined frame
        cv2.imshow('Combined Video Feed', reg_distort)

        # LAB and LUV channel threshold
        s_binary = binary_threshold_lab_luv(lane_test_undist, B_CHANNEL_THRESH, L2_CHANNEL_THRESH)

            # Convert the frame to grayscale
        gray = cv2.cvtColor(lane_test_undist, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)

    
    
    
        # Threshold the frame to get white pixels
        _, thresholded = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)

        cv2.imshow("COPE",thresholded)



        # Gradient threshold on S channel
        h, l, s = seperate_hls(lane_test_undist)
        sxbinary = gradient_threshold(s, GRADIENT_THRESH)


        # Combine two binary images to view their contribution in green and red
        color_binary = np.dstack((sxbinary, s_binary, np.zeros_like(sxbinary))) * 255

        
        # bottom_row = np.hstack((sxbinary, color_binary))

        # binary_images = np.vstack((top_row, bottom_row))

        # # Display the combined frame
        

        IMG_SIZE = lane_test_undist.shape[::-1][1:]
        OFFSET = 300

        white_pixel_detector = WhitePixelDetector(lane_test_undist)

        result = white_pixel_detector.process_frame()
        if result:
            frame, tl_pt, tr_pt, bl_pt, br_pt = result
            print("Top Left:", tl_pt)
            print("Top Right:", tr_pt)
            print("Bottom Left:", bl_pt)
            print("Bottom Right:", br_pt)
            



            PRES_SRC_PNTS = np.float32([
                tl_pt, # Top-left corner
                bl_pt, # Bottom-left corner
                br_pt, # Bottom-right corner
                tr_pt # Top-right corner
            ])

            PRES_DST_PNTS = np.float32([
                [OFFSET, 0], 
                [OFFSET, IMG_SIZE[1]],
                [IMG_SIZE[0]-OFFSET, IMG_SIZE[1]], 
                [IMG_SIZE[0]-OFFSET, 0] 
])
            M = cv2.getPerspectiveTransform(PRES_SRC_PNTS, PRES_DST_PNTS)
            M_INV = cv2.getPerspectiveTransform(PRES_DST_PNTS, PRES_SRC_PNTS)
            warped = cv2.warpPerspective(lane_test_undist, M, IMG_SIZE, flags=cv2.INTER_LINEAR)

            warped_cp = warped.copy()
            warped_poly = cv2.polylines(warped_cp, np.int32([PRES_DST_PNTS]), True, (255,0,0), 3)

        
            cv2.polylines(lane_test_undist, np.int32([PRES_SRC_PNTS]), True, (255, 0, 0), 3)
            top_row = np.hstack((lane_test_undist, color_binary))
            cv2.imshow('2x2 Video Configuration', top_row)
            ort_dots = np.hstack((frame, warped_cp))
            cv2.imshow('LANES', ort_dots)

            N_WINDOWS = 10
            MARGIN = 100
            RECENTER_MINPIX = 50

            # Define conversions in x and y from pixels space to meters
            YM_PER_PIX = 30 / 720 # meters per pixel in y dimension
            XM_PER_PIX = 3.7 / 700 # meters per pixel in x dimension





            # Warp binary image of lane line
            binary_warped = cv2.warpPerspective(s_binary, M, IMG_SIZE, flags=cv2.INTER_LINEAR)
            histogram = np.sum(binary_warped[int(binary_warped.shape[0]/2):,:], axis=0)
            print(histogram)

            # f, axarr = plt.subplots(2,2)
            # f.set_size_inches(18, 10)
            # axarr[0, 0].imshow(binary_warped, cmap='gray')
            # axarr[0, 1].plot(histogram)
            # axarr[0, 0].set_title("Warped Binary Lane Line")
            # axarr[0, 1].set_title("Histogram of Lane line Pixels")


            cv2.imshow("binary_warped",binary_warped)

            # leftx_base, rightx_base = histo_peak(histogram)
            # left_lane_inds, right_lane_inds, nonzerox, nonzeroy, out_img = get_lane_indices_sliding_windows(
            #     binary_warped, leftx_base, rightx_base, N_WINDOWS, MARGIN, RECENTER_MINPIX)


            # # Extract left and right line pixel positions
            # leftx = nonzerox[left_lane_inds]
            # lefty = nonzeroy[left_lane_inds] 
            # rightx = nonzerox[right_lane_inds]
            # righty = nonzeroy[right_lane_inds]


            # # Fit a second order polynomial to each
            # left_fit = np.polyfit(lefty, leftx, 2)
            # right_fit = np.polyfit(righty, rightx, 2)


            # # Generate x and y values for plotting
            # ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0])
            # left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            # right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

            # for y, left_x, right_x in zip(ploty.astype(int), left_fitx.astype(int), right_fitx.astype(int)):
            #     cv2.circle(out_img, (left_x, y), 2, (255, 0, 0), -1)  # Draw left line in red
            #     cv2.circle(out_img, (right_x, y), 2, (0, 255, 255), -1)  # Draw right line in yellow

            


            # # we may have to do some proccessing of our own. All it needs is something detected thats white. 
            # cv2.imshow("PLYFIT",out_img)











        # Break the loop when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
 
    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()




    # f, axarr = plt.subplots(2,2)
    # f.set_size_inches(17, 10)
    # axarr[0, 0].imshow(lane_test_img_rgb)
    # axarr[0, 1].imshow(lane_test_undist)
    # axarr[0, 0].set_title("Original Image")
    # axarr[0, 1].set_title("Undistorted Image")
    # axarr[0, 0].axis('off')
    # axarr[0, 1].axis('off')




    # Show the plots
    # plt.show()


if __name__ == "__main__":
    main()

