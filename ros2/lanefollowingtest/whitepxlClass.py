import cv2
import numpy as np
import math
from functools import partial

class WhitePixelDetector:
    def __init__(self,frame):
        # self.cap = cv2.VideoCapture(video_source)
        self.frame = frame
        self.tl = (0, 0)  # Top left
        self.tr = (640, 0)
        self.bl = (0, 480)
        self.br = (640, 480)
        self.tl_dist = partial(self.euclidean_distance, reference_point=self.tl)
        self.tr_dist = partial(self.euclidean_distance, reference_point=self.tr)
        self.bl_dist = partial(self.euclidean_distance, reference_point=self.bl)
        self.br_dist = partial(self.euclidean_distance, reference_point=self.br)

    def euclidean_distance(self, coord, reference_point):
        return math.sqrt((coord[0] - reference_point[0]) ** 2 + (coord[1] - reference_point[1]) ** 2)

    def detect_white_pixels(self):
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)
        _, thresholded = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        white_pixel_coordinates = [point[0] for contour in contours for point in contour]
        return white_pixel_coordinates

    def interpolate_and_draw_line(self,coordinates):
        if len(coordinates) > 1:
            coordinates_np = np.array(coordinates, dtype=np.int32)
            coordinates_np = coordinates_np.reshape(-1, 1, 2)
            cv2.polylines(self.frame, [coordinates_np], isClosed=True, color=(0, 0, 255), thickness=2)

    def process_frame(self):
        # ret, frame = self.cap.read()

        height, width, _ = self.frame.shape

        top_left = (0, 0)
        top_right = (width, 0)
        bottom_left = (0, height)
        bottom_right = (width, height)

        white_pixel_coordinates = self.detect_white_pixels()

        if white_pixel_coordinates:
            sorted_coordinates_distance_tl = sorted(white_pixel_coordinates, key=self.tl_dist)
            sorted_coordinates_distance_tr = sorted(white_pixel_coordinates, key=self.tr_dist)
            sorted_coordinates_distance_bl = sorted(white_pixel_coordinates, key=self.bl_dist)
            sorted_coordinates_distance_br = sorted(white_pixel_coordinates, key=self.br_dist)
            tl_pt = sorted_coordinates_distance_tl[0]
            tr_pt = sorted_coordinates_distance_tr[0]
            bl_pt = sorted_coordinates_distance_bl[0]
            br_pt = sorted_coordinates_distance_br[0]

            # Draw circles on the frame
            cv2.circle(self.frame, tuple(tl_pt), 12, (0, 255, 0), -1)
            cv2.circle(self.frame, tuple(tr_pt), 12, (0, 255, 0), -1)
            cv2.circle(self.frame, tuple(bl_pt), 12, (0, 255, 0), -1)
            cv2.circle(self.frame, tuple(br_pt), 12, (0, 255, 0), -1)

            # Interpolate to form a line and draw it on the frame
            self.interpolate_and_draw_line(white_pixel_coordinates)

            return self.frame, tl_pt, tr_pt, bl_pt, br_pt

        return None

    # def release_capture(self):
    #     self.cap.release()

# # Instantiate the class with the video source
# white_pixel_detector = WhitePixelDetector("/dev/video5")

# while True:
#     result = white_pixel_detector.process_frame()
#     if result:
#         frame, tl_pt, tr_pt, bl_pt, br_pt = result
#         print("Top Left:", tl_pt)
#         print("Top Right:", tr_pt)
#         print("Bottom Left:", bl_pt)
#         print("Bottom Right:", br_pt)
#         cv2.imshow('Interpolated Line', frame)

#     # Check for a key press and break the loop if 'q' is pressed
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # Release the webcam
# white_pixel_detector.release_capture()

# # Close all windows
# cv2.destroyAllWindows()
