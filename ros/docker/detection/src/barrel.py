import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import rospy
import os 
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
import numpy as np
from matplotlib import pyplot as plt


# DEFINES
# Subscribed Topics
IMAGE_TOPIC = '/zed/zed_node/rgb/image_rect_color'  
# rgb/image topic and depth topic come from left camera
# Depth map image registered on the left image (32-bit float in meters by default)
DEPTH_TOPIC = '/zed/zed_node/depth/depth_registered' # something like this

#Published Topics
BARREL_TOPIC = '/barrel_detection/rgb/image_rect_color'
BARREL_BOOL = '/barrel_bool'
BARREL_DIST = '/barrel_dist'

#PARAMETERS
BARREL_MAX_DIST = 3.5

bridge = CvBridge()

class Subscriber():
    def __init__(self):
        
        self.cv_img = cv.imread("../images/barrel.jpg")
        rospy.Subscriber(IMAGE_TOPIC, Image, self.img2cv)

    def img2cv(self,data):
        # rospy.loginfo('Video Frame Received')                                  
        # Converts image topic stream to cv image
        # Possible encoding schemes: 8UC[1-4], 8SC[1-4], 16UC[1-4], 16SC[1-4], 32SC[1-4], 32FC[1-4], 64FC[1-4]     
        # Can optionally do color or pixel depth conversion
        self.cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

class Publisher():
    def __init__(self):
        self.barrel_pub = rospy.Publisher(BARREL_TOPIC, Image, queue_size = 2)
        self.barrel_bool_pub = rospy.Publisher(BARREL_BOOL, Bool, queue_size = 2)
        self.barrel_dist_pub = rospy.Publisher(BARREL_DIST, Float32, queue_size = 2)
        self.rate = rospy.Rate(60)
    def pub_barrel(self,cv_image):
        img_msg = bridge.cv2_to_imgmsg(cv_image, "passthrough")
        self.barrel_pub.publish(img_msg)
        self.rate.sleep()
    def pub_dist(self,dist):
        self.barrel_dist_pub.publish(dist)
        self.rate.sleep()
    def pub_bool_barrel(self,distance):
        msg = Bool()
        if (distance < 100): #distance is less than 3 inches 
            msg.data = True 
        else:
            msg.data = False  
        self.barrel_bool_pub.publish(msg)


class Barrel():
    def __init__(self):
        self.source_img = cv.imread("../images/barrel.jpg")
        self.dist = 0

    def create_blob(self):
    # Set up the SimpleBlobDetector with default parameters
        params = cv.SimpleBlobDetector_Params()

        # Set the threshold
        params.minThreshold = 10
        params.maxThreshold = 255

        params.filterByColor = True
        params.blobColor = 255

        # Set the area filter
        params.filterByArea = False
        params.minArea = 10
        params.maxArea = 10000000
        #10,000 <--- for top of the bar

        # Set the circularity filter
        params.filterByCircularity = False
        params.minCircularity = 0.1
        params.maxCircularity = 1

        # Set the convexity filter
        params.filterByConvexity = False
        params.minConvexity = 0.87
        params.maxConvexity = 1

        # Set the inertia filter
        params.filterByInertia = False
        params.minInertiaRatio = 0.01
        params.maxInertiaRatio = 1

        detector = cv.SimpleBlobDetector_create(params)
        return detector



    def detection(self):
        #---------------------------(RESIZE_STUFF)---------------------------# 
        scale_percent = 20 # percent of original size
        width = int(self.source_img.shape[1] * scale_percent / 100)
        height = int(self.source_img.shape[0] * scale_percent / 100)
        dim = (width, height)
        barrel_jpg = cv.resize(self.source_img,dim, interpolation = cv.INTER_AREA)

        hsv = cv.cvtColor(barrel_jpg, cv.COLOR_BGR2HSV)
        # frame_thresh = cv.inRange(hsv,orange_min,orange_max)

        orange_min = (0, 97, 4)
        orange_max = (18, 255, 255)
        mask = cv.inRange(hsv,orange_min, orange_max)
        blur = cv.GaussianBlur(mask,(13,13),0)
        thresh = cv.threshold(blur, 100, 255, cv.THRESH_BINARY)[1]
        detector = self.create_blob()
        keypoints = detector.detect(thresh)
        im_with_keypoints = cv.drawKeypoints(barrel_jpg, keypoints, np.zeros((1,1)), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # cv.imshow("Keypoints", im_with_keypoints)
        # cv.waitKey(0)

        contours = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]
        cv.drawContours(im_with_keypoints, contours, -1, (0, 255, 0), 3)

        areas = [cv.contourArea(c) for c in contours]
        if (len(areas) != 0):
            max_index = np.argmax(areas)
            main_blob = contours[max_index] #biggest one --> closest to camera --> one we are interested in 

            x,y,w,h = cv.boundingRect(main_blob)
            self.source_img = cv.rectangle(im_with_keypoints,(x,y),(x+w,y+h),(255,0,0),2)
            # cv.imshow('Contours', im_with_keypoints)
            # cv.waitKey(1)
            # cv.destroyAllWindows()

            pxl_width = w #pixels
            known_dist = 60 #inches
            known_width = 24 #inches 
            ref_pxl = 208.0667266845703 
            #assuming 5 inches from barrel, using iphone camera scaled to 20 percent 
            #THIS PROBABLY NEEDS TO BE CHANGED 
            
            focal_ref = (ref_pxl * known_dist) / known_width
            dist_to_car = (known_width * focal_ref) / pxl_width

            print("Distance from car: ")
            print(dist_to_car/12)
            self.dist = dist_to_car/12
            #have to add an offset to this then divided by 12 
            return (dist_to_car/12)

        else:
            pass

def main():

    rospy.init_node('car_vision', anonymous=True)
    sub = Subscriber()
    pub = Publisher()
    barrel = Barrel()

    while not rospy.is_shutdown():
        barrel.source_img = sub.cv_img.copy()
        barrel.detection()
        
        pub.pub_barrel(barrel.source_img)
        pub.pub_dist(barrel.dist)
        pub.pub_bool_barrel(barrel.dist)

if __name__ == "__main__":
    main()
    