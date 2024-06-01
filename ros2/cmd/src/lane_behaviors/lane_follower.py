import math
import rclpy
from rclpy.node import Node

# from lane_behaviors.odom_sub import OdomSubscriber
from lane_behaviors.individual_follower import Individual_Follower
from lane_behaviors.controller.stanley import StanleyController

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from odom_sub import Odometry

class LaneFollower(Node):
    GUI = True
    # These are upper HSV & lower HSV bounds, respectively
    (l_h, l_s, l_v) = (120, 0, 240)
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
    # Kernel for blurring & removing "salt and pepper" noise
    KERNEL = 23
    LANE_TOLERANCE = 10

    # This is the lane follower Cstop/Estop trigger from crosstrack:
    # Effectively, it's an error beyond what our system is capable of returning, and we should trigger an Estop in the state machine if this value is ever read.
    MISSING_IMAGE_TOLERANCE = 100
    EMPTY_WINDOWS_THRESHOLD = 6
    OVERFLOW = 1000.0
    FORMAT = (640, 480)
    TOLERANCE = 100

    PIXELS_TO_METERS = 260.8269125

    def __init__(self, odom_sub: Odometry):
        super().__init__('lane_detection_node')

        # Inputs from both cameras
        self.vidcap_left = cv2.VideoCapture("/dev/video0")
        self.vidcap_right = cv2.VideoCapture("/dev/video2")
        # Setting the format for the images: we use 640 x 480
        self.vidcap_left.set(3, LaneFollower.FORMAT[0])
        self.vidcap_left.set(4, LaneFollower.FORMAT[1])
        self.vidcap_right.set(3, LaneFollower.FORMAT[0])
        self.vidcap_right.set(4, LaneFollower.FORMAT[1])

        self._Left_Lane = True
        self._tolerance = 0
        self._left_follower = Individual_Follower()
        self._right_follower = Individual_Follower()
        # Determine which lane we're in: Left lane means the right image is dashed

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Parameters for Stanley Controller
        self.heading_error = 0.0
        self.cross_track_error = 0.0
        # Flag used to signal invalid cross track error to transititon to ESTOP
        self.empty_error = False

        # Parameters for determining if object lies in lane or not.
        self.left_slope = 0.0
        self.right_slope = 0.0

        self.odom_sub = odom_sub
        self.stanley = StanleyController()

        if LaneFollower.GUI:
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

    def measure_position_meters(self, left, right, ignore_left=False, ignore_right=False):

        left_x_pos = 0
        right_x_pos = 0

        y_max = self._left_follower._binary_warped.shape[0]

        # Will be the same for left side & right sidmath.acos the bottom of the image
        if left is not None:
            left_fit = self._left_follower._fit
            # This is the first window coordinates
            # This is the second window coordinates
            left_x_pos = np.polyval(left_fit, y_max)
            self.img_publish("sliding_left", left)

        if right is not None:
            right_fit = self._right_follower._fit
            right_x_pos = np.polyval(right_fit, y_max)
            self.img_publish("sliding_right", right)
        width = self._left_follower._binary_warped.shape[1]

        if ignore_left:
            # 5 Feet = 1.52 Meters
            return 1.52 - right_x_pos*(LaneFollower.PIXELS_TO_METERS)
        if ignore_right:
            return 1.52 - left_x_pos*(LaneFollower.PIXELS_TO_METERS)

        center_lanes_x_pos = (left_x_pos + right_x_pos) // 2
        # Calculate the deviation between the center of the lane and the center of the picture
        # The car is assumed to be placed in the center of the picture
        # If the deviation is negative, the car is on the left hand side of the center of the lane
        veh_pos = ((width // 2) - center_lanes_x_pos) / \
            LaneFollower.PIXELS_TO_METERS

        return veh_pos

    # Used to calculate command from Stanley controller to stay in lane based on heading and cross track errors
    def follow_lane(self, period=0.05):

        steer_cmd = self.stanley.get_steering_cmd(
            self.heading_error,
            self.cross_track_error,
            # self.odom_sub.vel,
            2.235)  # comment this out when you want to use the actual velocity

        # vel_cmd = self.odom_sub.vel
        vel_cmd = 2.235  # Unsure if the velocity command should always be the target

        time.sleep(period)

        return steer_cmd, vel_cmd

    def timer_callback(self):
        success_l, image_l = self.vidcap_left.read()
        success_r, image_r = self.vidcap_right.read()

        # Left image is inverted
        image_l = cv2.rotate(image_l, cv2.ROTATE_180)
        # This may be necessary depending on the camera orientation: TEST FIRST
        # image_l = cv2.flip(image_l,0)

        # success_r, image_r = self.vidcap_right.read()
        # image_l = cv2.flip(image_l, 0)
        # Removing right image for testing only left camera
        images = [(image_l, "left"), (image_r, "right")]
        left_heading = -1
        right_heading = -1

        if not (success_l and success_r):
            return

        for image in images:
            frame = image[0]
            for point in (LaneFollower.bl,
                          LaneFollower.tl,
                          LaneFollower.br,
                          LaneFollower.tr):
                frame = cv2.circle(frame, point, 5, (0, 0, 255), -1)

            transformed_frame = cv2.rotate(
                cv2.warpPerspective(
                    frame, LaneFollower.UNWARP, LaneFollower.FORMAT),
                cv2.ROTATE_90_CLOCKWISE)
            # Object Detection
            # Image Thresholding

            blurred_preHSV = cv2.medianBlur(transformed_frame, self.KERNEL)

            hsv_transformed_frame = cv2.cvtColor(
                blurred_preHSV, cv2.COLOR_BGR2HSV
            )

            blurred = cv2.medianBlur(hsv_transformed_frame, self.KERNEL)
            mask = cv2.inRange(
                blurred, LaneFollower.LOWER, LaneFollower.UPPER
            )
            self.img_publish("mask_" + image[1], mask)

            if image[1] == "left":
                self._left_follower.set_binwarp(mask)
            else:
                self._right_follower.set_binwarp(mask)

            self.img_publish("raw_" + image[1], frame)
            self.img_publish("tf_" + image[1], transformed_frame)

        (result_left,
         empty_left,
         left_heading,
         left_slope) = self._left_follower.Plot_Line()
        (result_right,
         empty_right,
         right_heading,
         right_slope) = self._right_follower.Plot_Line()
        
        #Setting the slopes if there is enough relevant data: If not, we discard and assume what we're seeing is noise
        self.left_slope = left_slope * LaneFollower.PIXELS_TO_METERS if empty_left > LaneFollower.EMPTY_WINDOWS_THRESHOLD else self.left_slope
        self.right_slope = right_slope * LaneFollower.PIXELS_TO_METERS if empty_right > LaneFollower.EMPTY_WINDOWS_THRESHOLD else self.right_slope

        # TODO: Is this the behavior we want? Or do we need it to do something else if one of the lines is invalid?
        if result_left is not None or result_right is not None:
            ignore_left = True if empty_left > LaneFollower.EMPTY_WINDOWS_THRESHOLD else False
            ignore_right = True if empty_right > LaneFollower.EMPTY_WINDOWS_THRESHOLD else False

            cross_track = self.measure_position_meters(
                result_left, result_right, ignore_left, ignore_right)

            self.cross_track_error = cross_track
            self._Left_Lane = (True if empty_left <
                               empty_right - 2 else self._Left_Lane)
            self._Left_Lane = (False if empty_left - 2 >
                               empty_right else self._Left_Lane)
            # The slope calculations are returned in meters, must be adjusted.
            # main_msg.leftslope = left_slope / LaneFollower.PIXELS_TO_METERS
            # main_msg.rightslope = right_slope / LaneFollower.PIXELS_TO_METERS
            if self._Left_Lane:
                self.heading_error = left_heading
                # main_msg.leftlane = True
                # main_msg.he = left_heading
            else:
                self.heading_error = right_heading
                # main_msg.leftlane = False
                # main_msg.he = right_heading

        else:
            self._tolerance += 1
            if self._tolerance > LaneFollower.TOLERANCE:
                self.empty_error = True


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 program
    node = LaneFollower()  # Instantiate Node
    rclpy.spin(node)  # Puts the node in an infinite loop
    # Clean shutdown should be at the end of every node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
