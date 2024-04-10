import pyzed.sl as sl
import cv2
from ultralytics import YOLOWorld
import supervision as sv
import math
 
class ObjectDetection():
 
    def __init__(self,display=False):
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
        label_annotator = sv.LabelAnnotator(text_position=sv.Position.TOP_CENTER)
 
        bounding_box_img  = bounding_box_annotator.annotate(
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
            err, pointCloudVal = self.point_cloud.get_value(round(center_x),round(center_y))
            x_p = pointCloudVal[0]
            y_p = pointCloudVal[1]
            z_p = pointCloudVal[2]
    
            # print(z_p)
    
            #Find distance using Euclidean distance (in mm)
            distance = math.sqrt(x_p * x_p +
                        y_p * y_p +
                        z_p * z_p)
            
            self.distance = distance

        except IndexError: 
            self.distance = math.nan

        return self.distance
    
    def init_model(self):
        image_size = self.cam.get_camera_information().camera_configuration.resolution
 
        self.image_left_tmp = sl.Mat(self.cam.get_camera_information().camera_configuration.resolution.width, self.cam.get_camera_information().camera_configuration.resolution.height, sl.MAT_TYPE.U8_C4)
        self.point_cloud = sl.Mat(self.cam.get_camera_information().camera_configuration.resolution.width, self.cam.get_camera_information().camera_configuration.resolution.height, sl.MAT_TYPE.U8_C4)
 
        # win_name = "Camera Control"
        self.model=YOLOWorld('yolov8s-world.pt')
        self.model.to('cpu')
        self.model.set_classes(["water bottle"])

    def close_cam(self):
        cv2.destroyAllWindows()
        self.cam.close()
 
    def objectDetection(self):
        #grab frames from ZED
 
        # key = ''
        # while key != 113:
        err = self.cam.grab(self.runtime)
        if err == sl.ERROR_CODE.SUCCESS: # Check that a new image is successfully acquired
            # cam.retrieve_image(imageZed, sl.VIEW.LEFT) # Retrieve left image
            self.cam.retrieve_measure(self.point_cloud,sl.MEASURE.XYZRGBA)
            # cvImage = imageZed.get_data()
            self.cam.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
            image_net = self.image_left_tmp.get_data()
            self.img = cv2.cvtColor(image_net, cv2.COLOR_RGBA2RGB)
            cvPoint = self.point_cloud.get_data()
            results = self.model.predict(self.img)
            self.detections = sv.Detections.from_ultralytics(results[0])

        # else:
        #     print("Error during capture : ", err)
        #     break

        if self.display:
            self._display()

        # print((self.detections.empty()))

        dist = self.distance_to_objects()
            # print("Dist to object: ", self.detections.class_id[0],dist)   
            # key = cv2.waitKey(5)

        # self.close_cam()
 

 
# def main():
#     model = ObjectDetection(display=True)
#     model.init_model()
#     model.objectDetection()
 
 
 
# if __name__ == "__main__":
#     main()