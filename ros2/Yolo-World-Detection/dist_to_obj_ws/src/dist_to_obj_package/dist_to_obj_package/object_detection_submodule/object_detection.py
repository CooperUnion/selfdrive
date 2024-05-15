import pyzed.sl as sl
import cv2
from ultralytics import YOLOWorld
import supervision as sv
import math
import numpy as np


class ObjectDetection():

    def __init__(self, display=False):
        self.display = display
        init_params = sl.InitParameters()
        self.cam = sl.Camera()
        status = self.cam.open(init_params)

        # We should tune these vals
        init_params.coordinate_units = sl.UNIT.METER
        # For applications requiring long-range depth perception, they recommend setting depth_minimum_distance to 1m or more for improved performance.
        init_params.depth_minimum_distance = 15
        init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        init_params.depth_maximum_distance = 2000
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : "+repr(status)+". Exit program.")
            exit()
        self.runtime = sl.RuntimeParameters()
        # self.runtime.enable_fill_mode

    def _display(self):
        bounding_box_annotator = sv.BoundingBoxAnnotator()
        label_annotator = sv.LabelAnnotator(
            text_position=sv.Position.TOP_CENTER)

        bounding_box_img = bounding_box_annotator.annotate(
            scene=self.img,
            detections=self.detections
        )

        self.annotated_frame = label_annotator.annotate(
            scene=bounding_box_img,
            detections=self.detections
        )

        cv2.imshow("IMAGEE", self.annotated_frame)

    def distance_to_objects(self):

        # x = int(left + width/2)
        # y = int(top + height/2)

        try:
            box = self.detections.xyxy[0]
            center_x = int(box[0] + box[2])/2
            center_y = int(box[1] + box[3])/2
            err, pointCloudVal = self.point_cloud.get_value(
                round(center_x), round(center_y))
            x_p = pointCloudVal[0]
            y_p = pointCloudVal[1]
            z_p = pointCloudVal[2]

            # print(z_p)

            # Find distance using Euclidean distance (in mm)
            distance = math.sqrt(x_p * x_p +
                                 y_p * y_p +
                                 z_p * z_p)

            self.distance = distance
            return distance, (x_p, y_p, z_p)

        except IndexError:
            self.distance = math.nan
            return 0, (0,0,0)

    def init_model(self):

        self.image_left_tmp = sl.Mat(self.cam.get_camera_information().camera_configuration.resolution.width,
                                     self.cam.get_camera_information().camera_configuration.resolution.height, sl.MAT_TYPE.U8_C4)
        self.point_cloud = sl.Mat(self.cam.get_camera_information().camera_configuration.resolution.width,
                                  self.cam.get_camera_information().camera_configuration.resolution.height, sl.MAT_TYPE.U8_C4)

        # win_name = "Camera Control"
        self.model = YOLOWorld('yolov8s-world.pt')
        self.model.to('cpu')
        self.model.set_classes(["water bottle"])

    def close_cam(self):
        cv2.destroyAllWindows()
        self.cam.close()

    def objectDetection(self):
        # grab frames from ZED
        # key = ''
        # while key != 113:
        err = self.cam.grab(self.runtime)
        if err == sl.ERROR_CODE.SUCCESS:  # Check that a new image is successfully acquired
            # cam.retrieve_image(imageZed, sl.VIEW.LEFT) # Retrieve left image
            self.cam.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            # cvImage = imageZed.get_data()
            self.cam.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
            image_net = self.image_left_tmp.get_data()
            self.img = cv2.cvtColor(image_net, cv2.COLOR_RGBA2RGB)
            results = self.model.predict(self.img)
            self.detections = sv.Detections.from_ultralytics(results[0])
            lane_bounds = self.determine_in_lane()
            obj_center_pos = self.distance_to_objects()
            if(lane_bounds[0] < obj_center_pos < lane_bounds[1]):
                return obj_center_pos[1]
            return -1


        # self.close_cam()
    def determine_in_lane(self):
        # Get bottom slice of image for lane determination
        lane_determination_img = self.img[0: 400, :]

        width = lane_determination_img.shape[1]
        # If we know what lane we're in, we just need the center line:
        # This should be modified accordingly, with the lane following algorithm
        imgs = (lane_determination_img[:, :width//2],
                lane_determination_img[:, width//2:])
        # left is left bounds, right is right bounds, respectively
        centers = [self.get_index(img) for img in imgs]
        # Need to shift bound1 coordinates so that it corresponds accordingly for the pointcloud
        bound1 = (centers[0][0], centers[0][1] + width//2)
        bound2 = (centers[1][0], centers[1][1])
        #Getting only the y coordinate, only one that is relevant:
        coordinates = [self.point_cloud.get_value(
            c[0], c[1])[1] for c in (bound1, bound2)]
        #Sorting these points so that we have the minimum y & maximum y bounds
        return coordinates.sort()

    def get_index(self, img):
        imgray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        ret, thresh = cv2.threshold(imgray, 200, 255, 0)
        im2, contours, hierarchy = cv2.findContours(
            thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        centers = get_center(contours)
        return centers


def get_center(contours):
    areas = [cv2.contourArea(c) for c in contours]
    max_index = np.argmax(areas)
    main_blob = contours[max_index]
    M = cv2.moments(main_blob)
    # Check that the contour is not empty
    # (M["m00"] is the number of pixels in the contour)
    if M["m00"] <= 0:
        return None
    # Compute and return the center of mass of the contour
    center_row = round(M["m01"] / M["m00"])
    center_column = round(M["m10"] / M["m00"])
    return (center_row, center_column)
