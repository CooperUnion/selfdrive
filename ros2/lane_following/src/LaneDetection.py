# from irobot_create_msgs.msg import WheelTicks, WheelTicks
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String
from igvc_msgs.msg import StanleyInput
from Individual_Follower import Individual_Follower


class Lane_Follower(Node):
    GUI = True
    # These are upper HSV & lower HSV bounds, respectively
    (l_h, l_s, l_v) = (0, 0, 180)
    (u_h, u_s, u_v) = (255, 255, 255)

    LOWER = np.array([l_h, l_s, l_v])
    UPPER = np.array([u_h, u_s, u_v])

    # Coordinates for the 4 alignment points: again, should be handled by the UI
    bl = (12, 472)
    tl = (90, 8)
    br = (499, 475)
    tr = (435, 24)
    # Aplying perspective transformation
    pts1 = np.float32([tl, bl, tr, br])
    pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])

    # Matrix to warp the image for birdseye window
    UNWARP = cv2.getPerspectiveTransform(pts1, pts2)

    LANE_TOLERANCE = 10

    # This is the lane follower Cstop/Estop trigger from crosstrack:
    # Effectively, it's an error beyond what our system is capable of returning, and we should trigger an Estop in the state machine if this value is ever read.
    MISSING_IMAGE_TOLERANCE = 100
    OVERFLOW = 1000.0
    FORMAT = (640, 480)
    TOLERANCE = 100

    PIXELS_TO_METERS = 260.8269125

    def __init__(self):
        super().__init__('lane_detection_node')

        # Inputs from both cameras
        self.vidcap_right = cv2.VideoCapture("/dev/video4")
        self.vidcap_left = cv2.VideoCapture("/dev/video2")
        # Setting the format for the images: we use 640 x 480
        self.vidcap_left.set(3, Lane_Follower.FORMAT[0])
        self.vidcap_left.set(4, Lane_Follower.FORMAT[1])
        self.vidcap_right.set(3, Lane_Follower.FORMAT[0])
        self.vidcap_right.set(4, Lane_Follower.FORMAT[1])

        self._Left_Lane = True
        self._tolerance = 0
        self._left_follower = Individual_Follower()
        self._right_follower = Individual_Follower()
        # Determine which lane we're in: Left lane means the right image is dashed

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Publisher for error from the lane lines relative to the camera FOV
        # # 10 refers to the number of published references: standard value
        # self.crosstrack_pub = self.create_publisher(Float64, 'cross_track', 10)
        # self.lane_pub = self.create_publisher(String, "lane_state", 10)
        # self.heading_pub = self.create_publisher(Float64, "heading", 10)

        self.lane_error_pub = self.create_publisher(
            StanleyInput, "stanley_data", 10)

        if Lane_Follower.GUI:
            self._bridge = CvBridge()
            image_labels = (
                "raw_left",
                "raw_right",
                "tf_left",
                "tf_right",
                "sliding_left",
                "sliding_right",
                "mask_left",
                "mask_right",
            )
            self._publishers = {
                label: self.create_publisher(Image, "/" + label, 10)
                for label in image_labels
            }

    def img_publish(self, label, img_raw):
        if self.GUI:
            #            cv2.imshow(label,img_raw)
            self._publishers[label].publish(
                self._bridge.cv2_to_imgmsg(img_raw, encoding="passthrough")
            )

    def measure_position_meters(self, left, right):

        left_x_pos = 0
        right_x_pos = 0

        y_max = self._left_follower._binary_warped.shape[0]

        # Will be the same for left side & right sidmath.acos the bottom of the image
        if left is not None:
            left_fit = self._left_follower._fit
            # This is the first window coordinates
            left_x_pos = np.polyval(
                left_fit, y_max
            )  # This is the second window coordinates
            self.img_publish("sliding_left", left)

        if right is not None:
            right_fit = self._right_follower._fit
            right_x_pos = np.polyval(right_fit, y_max)
            self.img_publish("sliding_right", right)

        center_lanes_x_pos = (left_x_pos + right_x_pos) // 2
        # Calculate the deviation between the center of the lane and the center of the picture
        # The car is assumed to be placed in the center of the picture
        # If the deviation is negative, the car is on the left hand side of the center of the lane
        veh_pos = (
            (self._left_follower._binary_warped.shape[1] // 2)
            - center_lanes_x_pos
        ) / Lane_Follower.PIXELS_TO_METERS

        return veh_pos

    def timer_callback(self):
        success_l, image_l = self.vidcap_left.read()
        success_r, image_r = self.vidcap_right.read()
        image_r = cv2.flip(image_r, 0)
        images = [(image_l, "left"), (image_r, "right")]
        left_heading = -1
        right_heading = -1
        main_msg = StanleyInput()
        if not (success_l and success_r):
            return

        for image in images:
            frame = image[0]
            for point in (
                Lane_Follower.bl,
                Lane_Follower.tl,
                Lane_Follower.br,
                Lane_Follower.tr,
            ):
                frame = cv2.circle(frame, point, 5, (0, 0, 255), -1)

            transformed_frame = cv2.rotate(
                cv2.warpPerspective(
                    frame, Lane_Follower.UNWARP, Lane_Follower.FORMAT
                ),
                cv2.ROTATE_90_CLOCKWISE,
            )
            # Object Detection
            # Image Thresholding
            hsv_transformed_frame = cv2.cvtColor(
                transformed_frame, cv2.COLOR_BGR2HSV
            )
            mask = cv2.inRange(
                hsv_transformed_frame, Lane_Follower.LOWER, Lane_Follower.UPPER
            )
            self.img_publish("mask_" + image[1], mask)

            if image[1] == "left":
                self._left_follower.set_binwarp(mask)
            else:
                self._right_follower.set_binwarp(mask)

            self.img_publish("raw_" + image[1], frame)
            self.img_publish("tf_" + image[1], transformed_frame)

        result_left, empty_left, left_heading, left_slope = self._left_follower.Plot_Line()
        result_right, empty_right, right_heading, right_slope = self._right_follower.Plot_Line()
        # TODO: Is this the behavior we want? Or do we need it to do something else if one of the lines is invalid?
        if result_left is not None or result_right is not None:
            crosstrack = self.measure_position_meters(
                result_left, result_right)
            main_msg.CE = crosstrack

            self._Left_Lane = (
                True if empty_left < empty_right else self._Left_Lane
            )
            self._Left_Lane = (
                False if empty_left > empty_right else self._Left_Lane
            )

            if self._Left_Lane:
                main_msg.LEFTLANE = True
                main_msg.HE = left_heading
                main_msg.SLOPE = left_slope
            else:
                main_msg.LEFTLANE = False
                main_msg.HE = right_heading
                main_msg.SLOPE = right_slope

        else:
            self._tolerance += 1
            if self._tolerance > Lane_Follower.TOLERANCE:
                main_msg.ERROR = True

        self.lane_error_pub.publish(main_msg)


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 program
    node = Lane_Follower()  # Instantiate Node
    rclpy.spin(node)  # Puts the node in an infinite loop
    # Clean shutdown should be at the end of every node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
