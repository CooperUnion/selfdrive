

class Pedestrian():
    def __init__(self):
        self.source_img = cv.imread("../images/person.jpg")

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
        # frame_thresh = cv2.inRange(hsv,orange_min,orange_max)


        color_min = (0, 20, 4)
        color_max = (18, 60, 255)

        


        mask = cv.inRange(hsv,color_min, color_max)
        blur = cv.GaussianBlur(mask,(13,13),0)
        thresh = cv.threshold(blur, 100, 255, cv.THRESH_BINARY)[1]
        detector = self.create_blob()
        keypoints = detector.detect(thresh)
        im_with_keypoints = cv.drawKeypoints(barrel_jpg, keypoints, np.zeros((1,1)), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv.imshow("Keypoints", im_with_keypoints)
        cv.waitKey(0)

        contours = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[0]
        cv.drawContours(im_with_keypoints, contours, -1, (0, 255, 0), 3)

        areas = [cv.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        main_blob = contours[max_index] #biggest one --> closest to camera --> one we are interested in 

        x,y,w,h = cv.boundingRect(main_blob)
        cv.rectangle(im_with_keypoints,(x,y),(x+w,y+h),(255,0,0),2)
        cv.imshow('Contours', im_with_keypoints)
        cv.waitKey(1)
        cv.destroyAllWindows()

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
        return im_with_keypoints, (dist_to_car/12)