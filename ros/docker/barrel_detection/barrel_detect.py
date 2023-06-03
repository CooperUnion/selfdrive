import cv2 
import numpy as np 
import os 
from std_msgs.msg import Bool






class Barrel_Detection:
    def create_blob(self):
        # Set up the SimpleBlobDetector with default parameters
        params = cv2.SimpleBlobDetector_Params()

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

        detector = cv2.SimpleBlobDetector_create(params)
        return detector

    def orange_color (self,img):
        #---------------------------(RESIZE_STUFF)---------------------------# 
        scale_percent = 20 # percent of original size
        width = int(img.shape[1] * scale_percent / 100)
        height = int(img.shape[0] * scale_percent / 100)
        dim = (width, height)
        barrel_jpg = cv2.resize(img,dim, interpolation = cv2.INTER_AREA)

        hsv = cv2.cvtColor(barrel_jpg, cv2.COLOR_BGR2HSV)
        # frame_thresh = cv2.inRange(hsv,orange_min,orange_max)

        orange_min = (0, 97, 4)
        orange_max = (18, 255, 255)
        mask = cv2.inRange(hsv,orange_min, orange_max)
        blur = cv2.GaussianBlur(mask,(13,13),0)
        thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY)[1]
        detector = self.create_blob()
        keypoints = detector.detect(thresh)
        im_with_keypoints = cv2.drawKeypoints(barrel_jpg, keypoints, np.zeros((1,1)), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.imshow("Keypoints", im_with_keypoints)
        cv2.waitKey(0)

        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        cv2.drawContours(im_with_keypoints, contours, -1, (0, 255, 0), 3)

        areas = [cv2.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        main_blob = contours[max_index] #biggest one --> closest to camera --> one we are interested in 

        x,y,w,h = cv2.boundingRect(main_blob)
        cv2.rectangle(im_with_keypoints,(x,y),(x+w,y+h),(255,0,0),2)
        cv2.imshow('Contours', im_with_keypoints)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

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
        #have to add an offset to this then divided by 12 
        return (dist_to_car/12)
    
    def publisher(self): 
        pass 
    
    
class pub_barrel:
    def __init__(self):
        self.bool_barrel = Bool
        self.pub = rospy.Publisher('/bool_barrel',Bool, queue_size = 2)
        self.rate = rospy.rate(2) #2G Hz

    def publish(self): 
            folder = "Barrel_Images"
            file = "Barrel_3.jpg"
            img = cv2.imread(os.path.join(folder,file))
            Barrel_D = Barrel_Detection()
            dist_to_thing = Barrel_D.orange_color(img)

            if (dist_to_thing < 3.5): #distance is less than 3 inches 
                self.msg.Bool = True 
            else:
                self.msg.Bool = False  
                 
            while not rospy.is_shutdown():
                    self.pub.publish(self.bool_barrel)
                    
def main():    
    folder = "Barrel_Images"
    file = "Barrel_3.jpg"
    img = cv2.imread(os.path.join(folder,file))
    Barrel_D = Barrel_Detection()
    dist_to_thing = Barrel_D.orange_color(img)
    print(dist_to_thing)

if __name__ == "__main__":
     rospy.init_node('bool_barrel',anoymous = True)
     publisher = pub_barrel()
     publisher.publish()
     rospy.spin()

    #reference is 5ft from the zed to the barrel 