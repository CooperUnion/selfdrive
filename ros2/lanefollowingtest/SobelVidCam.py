import numpy as np
import cv2
import matplotlib.pyplot as plt
from moviepy.editor import VideoFileClip
from queue import Queue
import os
import glob
from whitepxlClass import WhitePixelDetector



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
        # print(img_locs)

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
    

def warp_image(img,src,dst):
    img_size = (img.shape[1], img.shape[0])
    M= cv2.getPerspectiveTransform(src, dst) 
    inv= cv2.getPerspectiveTransform(dst, src)
    warped= cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)
    return warped,inv

def reverse_warping(img,M):
    img_size = (img.shape[1], img.shape[0])
    unwarped= cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)
    return unwarped

def channelwise_thresholding(image,thresh):
    image = image*(255/np.max(image))
    # print(image)
    # 2) Apply a threshold to the L channel
    binary_output = np.zeros_like(image)
    binary_output[(image > thresh[0]) & (image <= thresh[1])] = 1
    return binary_output

def Custom_channel_converter(img):
    
    img1=cv2.cvtColor(img,cv2.COLOR_RGB2YCrCb)[:,:,0] # Y channel
    img2=cv2.cvtColor(img,cv2.COLOR_RGB2YCrCb)[:,:,1] #Cr channel
    img3=cv2.cvtColor(img,cv2.COLOR_RGB2HLS)[:,:,1] #L channel
    img4=cv2.cvtColor(img,cv2.COLOR_RGB2HLS)[:,:,2] #S channel
    return img1, img2, img3, img4

def sobel_image(img, orient='x', thresh_min=0, thresh_max=255, convert=True):
    
    
    gray= img
    if(convert):
        gray= cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    
    sobel=None
    if(orient=='x'):
        sobel= cv2.Sobel(gray, cv2.CV_64F, 1,0)
    else:
        sobel= cv2.Sobel(gray, cv2.CV_64F, 0,1)
    
    sobel_abs= np.absolute(sobel)
    sobel_8bit= np.uint8(255* sobel_abs/np.max(sobel_abs))
    binary_output= np.zeros_like(sobel_8bit) 
    binary_output[(sobel_8bit>=thresh_min) & (thresh_max>=sobel_8bit)]=1
    
    return binary_output

def sobel_gradient_image(img, thresh=(0, np.pi/2), convert=True):
    gray= img
    if(convert):
        gray= cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    
    sobelx= cv2.Sobel(gray, cv2.CV_64F, 1,0, ksize=15)
    sobely= cv2.Sobel(gray, cv2.CV_64F, 0,1, ksize=15)
    
    abs_sobelx= np.absolute(sobelx)
    abs_sobely= np.absolute(sobely)
    
    grad= np.arctan2(abs_sobely, abs_sobelx)
    
    binary_output=np.zeros_like(grad)
    binary_output[(grad>thresh[0])&(grad<thresh[1])]=1
    return binary_output


mtx = None
dist = None
fit_prev_left=[]
fit_prev_right=[]
fit_sum_left=0
fit_sum_right=0
def Plot_line(binary_warped, smoothen=False,prevFrameCount=10): #used Udacity's code to plot the lines and windows over lanes 
    histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
    # Create an output image to draw on and  visualize the result
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int32(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    lane_width= abs(rightx_base-leftx_base)
    # Choose the number of sliding windows
    nwindows = 9
    # Set height of windows
    window_height = np.int32(binary_warped.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    # Set the width of the windows +/- margin
    margin = 100
    # Set minimum number of pixels found to recenter window
    minpix = 50
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
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
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 

    # Fit a second order polynomial to each
    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)
    
    if(smoothen):
        global fit_prev_left
        global fit_prev_right
        global fit_sum_left
        global fit_sum_right
        if(len(fit_prev_left)>prevFrameCount):
            fit_sum_left-= fit_prev_left.pop(0)
            fit_sum_right-= fit_prev_right.pop(0)

        fit_prev_left.append(left_fit)
        fit_prev_right.append(right_fit)
        fit_sum_left+=left_fit
        fit_sum_right+= right_fit

        no_of_fit_values=len(fit_prev_left) 
        left_fit= fit_sum_left/no_of_fit_values
        right_fit= fit_sum_right/no_of_fit_values
    
    
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    
    nonzero = binary_warped.nonzero()
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
    cv2.fillPoly(window_img, np.int32([left_line_pts]), (0,255, 0))
    cv2.fillPoly(window_img, np.int32([right_line_pts]), (0,255, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

    return out_img, result, left_fitx,right_fitx,ploty,left_fit, right_fit,left_lane_inds,right_lane_inds,lane_width

def draw_lane(original_img, Combined_img, left_fitx, right_fitx, M):
    new_img = np.copy(original_img)
    warp_zero = np.zeros_like(Combined_img).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
    
    h,w = Combined_img.shape
    ploty = np.linspace(0, h-1, num=h)

    # print("left_fitx shape:", left_fitx.shape)
    # print("ploty shape:", ploty.shape)

    if len(left_fitx) != len(ploty):
        # Adjust the length of left_fitx to match the length of ploty
        left_fitx = left_fitx[:len(ploty)]

    if len(right_fitx) != len(ploty):
        # Adjust the length of left_fitx to match the length of ploty
        right_fitx = right_fitx[:len(ploty)]

    
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))
    cv2.polylines(color_warp, np.int32([pts_left]), isClosed=False, color=(255,0,0), thickness=15)
    cv2.polylines(color_warp, np.int32([pts_right]), isClosed=False, color=(255,0,0), thickness=15)

    return color_warp, new_img

center_distances= Queue(maxsize=15)
distanceSum=0
def get_car_position(l_fit, r_fit,w,h):
    xm_per_pix=3.7/700

    center_dist=0
    lane_center_position=0
    if r_fit is not None and l_fit is not None:
        car_position = w/2
        l_fit_x_int = l_fit[0]*h**2 + l_fit[1]*h + l_fit[2]
        r_fit_x_int = r_fit[0]*h**2 + r_fit[1]*h + r_fit[2]
        lane_center_position = (r_fit_x_int + l_fit_x_int) /2
        center_dist = (car_position - lane_center_position) * xm_per_pix

    
    global distanceSum           
    if(center_distances.full()):
        el=center_distances.get()
        distanceSum-=el
    
    center_distances.put(center_dist)
    distanceSum+=center_dist
    
    no_of_distance_values=center_distances.qsize() 
    center_dist= distanceSum/no_of_distance_values
    return center_dist,lane_center_position

def get_direction(center_dist):
    direction = ''
    if center_dist > 0:
        direction = 'right'
    elif center_dist < 0:
        direction = 'left'
    return direction

def Plot_details(laneImage,curv_rad,center_dist,width_lane,lane_center_position):
    offest_top=0
    copy= np.zeros_like(laneImage)
    
    h = laneImage.shape[0]
    font = cv2.FONT_HERSHEY_COMPLEX_SMALL
    text = 'Curve radius: ' + '{:04.2f}'.format(curv_rad) + 'cm'
    cv2.putText(laneImage, text, (40,70+offest_top), font, 1.5, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(copy, text, (40,100+offest_top), font, 4.0, (255,255,255), 3, cv2.LINE_AA)
    
    abs_center_dist = abs(center_dist)
    direction= get_direction(center_dist)
    text = '{:04.3f}'.format(abs_center_dist) + 'm ' + direction + ' of center'
#     cv2.putText(laneImage, 'steering '+direction, (40,110+offest_top), font, 1.5, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(laneImage, '|', (640,710), font, 2.0, (255,255,255), 3, cv2.LINE_AA)
    cv2.putText(laneImage, '|', (int(lane_center_position),680), font, 2.0, (255,0,0), 3, cv2.LINE_AA)
    cv2.putText(laneImage, text, (40,120+offest_top), font, 1.5, (255,255,255), 2, cv2.LINE_AA)
    
    text = 'Lane Width: ' + '{:04.2f}'.format(width_lane) + 'm'
    cv2.putText(laneImage, text, (40,170+offest_top), font, 1.5, (255,255,255), 2, cv2.LINE_AA)
    cv2.putText(copy, text, (40,280+offest_top), font, 4.0, (255,255,255), 3, cv2.LINE_AA)
    
    return laneImage, copy

width_lane_avg=[]
radius_values = Queue(maxsize=15)
radius_sum=0

def calc_radius_position(combined, l_fit, r_fit, l_lane_inds, r_lane_inds,lane_width):
    
    # Define conversions in x and y from pixels space to meters
    ym_per_pix = (30*100)/720 # meters per pixel in y dimension
    xm_per_pix = (3.7*100)/700 # meters per pixel in x dimension
    left_curverad, right_curverad, center_dist, width_lane = (0, 0, 0, 0)
    h = combined.shape[0]
    w = combined.shape[1]
    ploty = np.linspace(0, h-1, h)
    y_eval = np.max(ploty)
  
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = combined.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    
    # Extract left and right line pixel positions
    leftx = nonzerox[l_lane_inds]
    lefty = nonzeroy[l_lane_inds] 
    rightx = nonzerox[r_lane_inds]
    righty = nonzeroy[r_lane_inds]
    
    if len(leftx) != 0 and len(rightx) != 0:
        # Fit new polynomials to x,y in world space
        left_fit_cr = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
        right_fit_cr = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
        
        #applying the formula for 
        left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
        
        width_lane= lane_width*xm_per_pix
        if(len(width_lane_avg) != 0):
            avg_width=(sum(width_lane_avg)/len(width_lane_avg))
            if abs(avg_width-width_lane)<0.5:
                width_lane_avg.append(width_lane)
            else:
                width_lane=avg_width
    
    
    # Averaging radius value over past 15 frames
    global radius_sum           
    if(radius_values.full()):
        el=radius_values.get()
        
        radius_sum-=el
    curve_radius= (left_curverad+right_curverad)/2
    radius_values.put(curve_radius)
    radius_sum+=curve_radius
    
    no_of_radius_values=radius_values.qsize() 
    curve_radius= radius_sum/no_of_radius_values
#     print(curve_radius, radius_sum,no_of_radius_values)
    
    center_dist,lane_center_position= get_car_position(l_fit,r_fit,w,h) #getting the car distance from the center
    return curve_radius, center_dist,width_lane,lane_center_position

def undistort(img,mtx,dist):
    return cv2.undistort(img,mtx,dist, None, mtx)


def Lane_pipeline(img,smoothen,prevFrameCount):
    undistorted_image= undistort(img,mtx,dist)
    warped_image,M= warp_image(undistorted_image)
    image_S_channel= cv2.cvtColor(warped_image, cv2.COLOR_RGB2HLS)[:,:,2]
    
    imgY, imgCr, imgb, imgS= Custom_channel_converter(warped_image)
    
    Ybinary= channelwise_thresholding(imgY,(215,255))
    Crbinary= channelwise_thresholding(imgCr,(215,255))
    Lbinary= channelwise_thresholding(imgb,(215,255))
    Sbinary= channelwise_thresholding(imgS,(200,255))
    combined = np.zeros_like(imgY)
    
    
#     sobel_mag_image= sobel_mag(image_S_channel, (15,60), False)
    # sobel_image1= sobel_image(image_S_channel,'x', 15,60, False)
    # sobel_grad_image= sobel_gradient_image(image_S_channel,  (0.5,1.8), False)
    # combined[(Crbinary==1)|(Ybinary==1)|((Lbinary==1)&(Sbinary==1))] = 1
#     |((sobel_image1==1) & (sobel_grad_image==1))
#     plt.imshow(combined)
#     combined[]=1
    
#     |((sobel_image1==1)&(sobel_grad_image==1))
#     ((sobel_mag_image == 1) & (sobel_grad_image == 0))
    
#     out_img,out_img1, left_fitx,right_fitx,ploty,left_curverad,right_curverad,center_dist= Plot_line(combined)
    out_img,out_img1, left_fitx,right_fitx,ploty,left_fit, right_fit,left_lane_inds,right_lane_inds,lane_width= Plot_line(Ybinary,smoothen,prevFrameCount)
    curverad,center_dist,width_lane,lane_center_position= calc_radius_position(combined,left_fit, right_fit,left_lane_inds,right_lane_inds,lane_width)
    laneImage,new_img =draw_lane(img, combined, left_fitx, right_fitx, M)
    unwarped_image= reverse_warping(laneImage,M)
    laneImage = cv2.addWeighted(new_img, 1, unwarped_image, 0.5, 0)
    laneImage, copy = Plot_details(laneImage,curverad,center_dist,width_lane,lane_center_position)
    # print(center_dist)
    return img,out_img,out_img1,unwarped_image,laneImage,combined,copy

    

def CallPipeline(image):
    smoothen= True
    prevFrameCount=4
    rgb_image,out_img,out_img1,unwarped_image,laneImage,combined,data_copy= Lane_pipeline(image,smoothen,prevFrameCount)

    out_image = np.zeros((720,1280,3), dtype=np.uint8)
    
    #stacking up various images in one output Image
    out_image[0:720,0:1280,:] = cv2.resize(laneImage,(1280,720)) #top-left
    out_image[20:190,960:1260,:] = cv2.resize(np.dstack((combined*255, combined*255, combined*255)),(300,170))#side Panel
    out_image[210:380,960:1260,:] = cv2.resize(out_img,(300,170))#side Panel
#     out_image[400:570,960:1260,:] = cv2.resize(data_copy,(300,170))#bottom-left
    return out_image


       
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
    mtx = MTX
    dist = DIST


    ret = True
    while ret:
        ret, frame = cap.read()
        lane_test_img = frame

        offset=10
        height, width= frame.shape[0], frame.shape[1]

        lane_test_undist = cv2.undistort(lane_test_img, MTX, DIST, None, MTX)
        # lane_test_undist = cv2.cvtColor(lane_test_img, cv2.COLOR_BGR2RGB)
        reg_distort = np.hstack((frame, lane_test_undist))
        white_pixel_detector = WhitePixelDetector(lane_test_undist)

        

        result = white_pixel_detector.process_frame()
        if result:
            frame, tl_pt, tr_pt, bl_pt, br_pt = result
            # print("Top Left:", tl_pt)
            # print("Top Right:", tr_pt)
            # print("Bottom Left:", bl_pt)
            # print("Bottom Right:", br_pt)

            src = np.float32([
            tl_pt, # Top-left corner
            bl_pt, # Bottom-left corner
            br_pt, # Bottom-right corner
            tr_pt # Top-right corner
            ])
            
            dst=np.float32([(offset,0),(width-offset,0),(width-offset,height),(offset,height)])

            unwarped_image,M= warp_image(lane_test_img,src,dst)
            cv2.imshow("Bird's Eye View", unwarped_image)

            imgY, imgCr, imgL, imgS = Custom_channel_converter(unwarped_image)

            result_image = np.zeros((2 * imgY.shape[0], 2 * imgY.shape[1]), dtype=np.uint8)
            

            # Place each frame in the result image
            result_image[0:imgY.shape[0], 0:imgY.shape[1]] = imgY
            result_image[0:imgCr.shape[0], imgY.shape[1]:] = imgCr
            result_image[imgY.shape[0]:, 0:imgL.shape[1]] = imgL
            result_image[imgY.shape[0]:, imgL.shape[1]:] = imgS


            # Display labels on each section
            cv2.putText(result_image, 'imgY', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            cv2.putText(result_image, 'imgCr', (imgY.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            cv2.putText(result_image, 'imgL', (10, imgY.shape[0] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            cv2.putText(result_image, 'imgS', (imgY.shape[1] + 10, imgY.shape[0] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)



            Ybinary= channelwise_thresholding(imgY,(220,255))
            Crbinary= channelwise_thresholding(imgCr,(220,255))
            Lbinary= channelwise_thresholding(imgL,(220,252))
            Sbinary= channelwise_thresholding(imgS,(220,255))

            Ybinary= cv2.bitwise_not(Ybinary)
            Crbinary= cv2.bitwise_not(Crbinary)
            Lbinary= cv2.bitwise_not(Lbinary)
            Sbinary= cv2.bitwise_not(Sbinary)
            combined = np.zeros_like(imgY)

            warped_image,M= warp_image(lane_test_img,src,dst)

            image_S_channel= cv2.cvtColor(warped_image, cv2.COLOR_RGB2HLS)[:,:,2]

            sobel_image1= sobel_image(image_S_channel,'x', 15,60, False)
            sobel_grad_image= sobel_gradient_image(image_S_channel,  (0.5,1.8), False)
            combined[(Crbinary==1)|(Ybinary==1)|((Lbinary==1)&(Sbinary==1))] = 1
            combined = cv2.bitwise_not(combined)

            cv2.imshow("PLS",combined)

            

            # Create a blank image to display the binary images
            result_binary_image = np.zeros((2 * Ybinary.shape[0], 2 * Ybinary.shape[1]), dtype=np.uint8)

            # Place each binary image in the result image
            result_binary_image[0:Ybinary.shape[0], 0:Ybinary.shape[1]] = Ybinary
            result_binary_image[0:Crbinary.shape[0], Ybinary.shape[1]:] = Crbinary
            result_binary_image[Ybinary.shape[0]:, 0:Lbinary.shape[1]] = Lbinary
            result_binary_image[Ybinary.shape[0]:, Lbinary.shape[1]:] = Sbinary

            # Display labels on each section
            cv2.putText(result_binary_image, 'Ybinary', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            cv2.putText(result_binary_image, 'Crbinary', (Ybinary.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            cv2.putText(result_binary_image, 'Lbinary', (10, Ybinary.shape[0] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
            cv2.putText(result_binary_image, 'Sbinary', (Ybinary.shape[1] + 10, Ybinary.shape[0] + 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)

            # Display the result binary image
            cv2.imshow('Four Binary Images Display', result_binary_image)
            cv2.imshow('Color Spaces', result_image)

            global fit_prev_left
            global fit_prev_right
            global fit_sum_left
            global fit_sum_right
            fit_prev_left=[]
            fit_prev_right=[]
            fit_sum_left=0
            fit_sum_right=0

            out_img, out_img1, left_fitx,right_fitx,ploty,left_fit, right_fit,left_lane_inds,right_lane_inds,lane_width = Plot_line(Ybinary)

            
            # Generate x and y values for plotting
            ploty = np.linspace(0, result_binary_image.shape[0]-1, result_binary_image.shape[0])
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

            for y, left_x, right_x in zip(ploty.astype(int), left_fitx.astype(int), right_fitx.astype(int)):
                cv2.circle(out_img, (left_x, y), 2, (255, 0, 0), -1)  # Draw left line in red
                cv2.circle(out_img, (right_x, y), 2, (0, 255, 255), -1)  # Draw right line in yellow

            # we may have to do some proccessing of our own. All it needs is something detected thats white. 
            cv2.imshow("PLYFIT",out_img)

            curverad,center_dist,width_lane,lane_center_position= calc_radius_position(Ybinary,left_fit, right_fit,left_lane_inds,right_lane_inds,lane_width)
            laneImage,new_img =draw_lane(frame, Ybinary, left_fitx, right_fitx, M)
            unwarped_image= reverse_warping(laneImage,M)
            laneImage = cv2.addWeighted(new_img, 1, unwarped_image, 0.5, 0)
            laneImage, copy = Plot_details(laneImage,curverad,center_dist,width_lane,lane_center_position)
            print(center_dist)

            cv2.imshow("UMMA",laneImage)
        
        # # Display the combined frame
        # cv2.imshow('Combined Video Feed', reg_distort)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break





 
    # Release the webcam and close all windows
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()