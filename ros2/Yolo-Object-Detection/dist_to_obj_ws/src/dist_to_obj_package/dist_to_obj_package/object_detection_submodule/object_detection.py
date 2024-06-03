import numpy as np
import pyzed.sl as sl
import cv2
from ultralytics import YOLO
import supervision as sv
import math
from ..ocr_submodule.ocr import OCR


class ObjectDetection:

    def __init__(self, display=True):
        self.display = display
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 30

        self.cam = sl.Camera()

        self.distance = 0.0
        self.obj_x = 0.0
        self.obj_y = 0.0
        self.obj_name = ''

        # We should tune these vals
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = (
            sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        )
        # For applications requiring long-range depth perception, they recommend setting depth_minimum_distance to 1m or more for improved performance.
        init_params.depth_minimum_distance = 1
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.depth_stabilization = 500
        

        init_params.depth_maximum_distance = 10

        status = self.cam.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(status) + ". Exit program.")
            exit()
        self.runtime = sl.RuntimeParameters()

        # \self.runtime.enable_fill_mode = False
        # self.runtime.enable_fill_mode

    def init_model(self):
        image_size = (
            self.cam.get_camera_information().camera_configuration.resolution
        )

        self.image_left_tmp = sl.Mat(
            self.cam.get_camera_information().camera_configuration.resolution.width,
            self.cam.get_camera_information().camera_configuration.resolution.height,
            sl.MAT_TYPE.U8_C4,
        )
        self.point_cloud = sl.Mat(
            self.cam.get_camera_information().camera_configuration.resolution.width,
            self.cam.get_camera_information().camera_configuration.resolution.height,
            sl.MAT_TYPE.U8_C4,
        )

        self.model = YOLO('cooper.pt')
        self.model.to('cpu')

        self.ocr = OCR()

    def object_bounding_box(self):
        print("Getting bounding boxes")
        bounding_box_annotator = sv.BoundingBoxAnnotator()
        label_annotator = sv.LabelAnnotator(
            text_position=sv.Position.TOP_CENTER
        )

        bounding_box_img = bounding_box_annotator.annotate(
            scene=self.img, detections=self.detections
        )

        self.annotated_frame = label_annotator.annotate(
            scene=bounding_box_img, detections=self.detections
        )

        if not (len(self.detections) == 0):
            for detection in self.detections:
                bbox = detection[0]
                dist = self.distance_to_objects(bbox)
                self.obj_name = self.model.names[detection[3]]

                if "sign" in self.ocr.output:
                    self.obj_name = self.ocr.output
                else:
                    self.obj_name = self.model.names[detection[3]]

                distance_text = f"Distance: {dist:.2f} meters"
                confidence_text = f"Confidence: {detection[2]:.2f}"
                combined_text = f"{distance_text}\n{confidence_text}"

                # Add the combined text to the annotated frame
                cv2.putText(
                    self.annotated_frame,
                    combined_text,
                    (int(bbox[0]), int(bbox[1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
        # this should be the default output when no detections occur
        else:
            self.distance = -1.0
            self.obj_name = "No Object"
            self.obj_x = 0.0
            self.obj_y = 0.0

        cv2.imshow("Annotated Image", self.annotated_frame)
        cv2.waitKey(1)

    def distance_to_objects(self, box):
        # try:
        # box = self.detections.xyxy[0]
        center_x = int(box[0] + box[2]) / 2

        centers = np.empty((10, 4), dtype=np.float64)
        stepy = (box[3] - box[1]) / 10
        for i in range(10):
            err, centers[i] = self.point_cloud.get_value(
                round(center_x), round(box[1] + i * stepy)
            )
        #                centers = np.nan_to_num(centers,nan=0,posinf=0,neginf=0)
        # np.where(centers < 0, centers, np.inf).argmax()

            x_p = np.nanmean(centers[5,0])
            y_p = np.nanmean(centers[5,1])
            z_p = np.nanmean(centers[5,2])

        # x_p = np.nan_to_num(x_p,nan=-1,posinf=-1,neginf=-1)
        # y_p = np.nan_to_num(y_p,nan=-1,posinf=-1,neginf=-1)
        # z_p = np.nan_to_num(z_p,nan=-1,posinf=-1,neginf=-1)
        if x_p != np.nan and y_p != np.nan and z_p != np.nan:
            self.distance = math.sqrt(x_p**2 + y_p**2 + z_p**2)
            self.obj_x = x_p
            self.y_p = y_p

        # # err, pointCloudVal = self.point_cloud.get_value(
        # #     round(center_x), round(center_y)
        # # )
        # x_p, y_p, z_p = centers[0],
        # x_p = pointCloudVal[0]
        # y_p = pointCloudVal[1]
        # z_p = pointCloudVal[2]

        # except RuntimeWarning:
        #     print("Runtime Error Handled!")
        #     self.distance = -1.0
        #     self.obj_x = -1.0
        #     self.obj_y = -1.0

        # except IndexError:
        #     self.distance = -1.0
        #     self.obj_name = "No Object"
        #     self.obj_x = -1.0
        #     self.obj_y = -1.0

        return self.distance

    def close_cam(self):
        cv2.destroyAllWindows()
        self.cam.close()

    def objectDetection(self):
        # grab frames from ZED
        err = self.cam.grab(self.runtime)
        if (
            err == sl.ERROR_CODE.SUCCESS
        ):  # Check that a new image is successfully acquired
            # cam.retrieve_image(imageZed, sl.VIEW.LEFT) # Retrieve left image
            self.cam.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
            # cvImage = imageZed.get_data()
            self.cam.retrieve_image(self.image_left_tmp, sl.VIEW.LEFT)
            image_net = self.image_left_tmp.get_data()
            self.img = cv2.cvtColor(image_net, cv2.COLOR_RGBA2RGB)
            self.ocr(self.img)
            cvPoint = self.point_cloud.get_data()

            # # Crops the current image to view 1/3 of the frame
            # center = self.img.shape
            # x = center[1]/2
            # y = center[0]/2
            # t = 500
            # self.img = self.img[int(y):int(y+t), int(x):int(x+t)]
            results = self.model.predict(self.img)
            self.detections = sv.Detections.from_ultralytics(results[0])
            self.object_bounding_box()
