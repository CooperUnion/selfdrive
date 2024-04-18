
# from irobot_create_msgs.msg import WheelTicks, WheelTicks
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import String

# Inputs from both cameras
vidcap_left = cv2.VideoCapture("/dev/video3")
vidcap_left.set(3, 640)
vidcap_left.set(4, 480)
vidcap_right = cv2.VideoCapture("/dev/video1")
vidcap_right.set(3, 640)
vidcap_right.set(4, 480)


# These are constants
nwindows = 9
# Set the width of the windows +/- margin
margin = 100
# Set minimum number of pixels found to recenter window
minpix = 20

xm_per_pix = 1  # meters per pixel in x dimension


def nothing(x):
    pass


class Individual_Follower():

    def __init__(self):
        self._fit = None
        self._binary_warped = None

    def set_binwarp(self, binwarp):
        self._binary_warped = binwarp

    def Plot_Line(self, smoothen=False, prevFrameCount=6):
        histogram = np.sum(
            self._binary_warped[self._binary_warped.shape[0]//2:, :], axis=0)
        # Create an output image to draw on and  visualize the result
        out_img = np.dstack(
            (self._binary_warped, self._binary_warped, self._binary_warped))*255
        base = np.argmax(histogram[:])

        window_height = np.int32(self._binary_warped.shape[0]/nwindows)
        # Create empty list to hold pixel indices
        lane_inds = []

        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = self._binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        current = base

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = self._binary_warped.shape[0] - (window+1)*window_height
            win_y_high = self._binary_warped.shape[0] - window*window_height
            win_x_low = current - margin
            win_x_high = current + margin
            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high),
                          (0, 255, 0), 2)
            # Identify the nonzero pixels in x and y within the window
            good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                         (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
            # Append these indices to the lists
            lane_inds.append(good_inds)
            if len(good_inds) > minpix:
                current = np.int32(np.mean(nonzerox[good_inds]))

        # Concatenate the arrays of indices
        if (len(lane_inds) > 0):
            lane_inds = np.concatenate(lane_inds)

        # Extract line pixel positions
        x_pos = nonzerox[lane_inds]
        y_pos = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        if (x_pos.any() and y_pos.any()):
            self._fit = np.polyfit(y_pos, x_pos, 2)
        else:
            return None

        ploty = np.linspace(
            0, self._binary_warped.shape[0]-1, self._binary_warped.shape[0])
        fitx = self._fit[0]*ploty**2 + \
            self._fit[1]*ploty + self._fit[2]
        out_img[nonzeroy[lane_inds],
                nonzerox[lane_inds]] = [255, 0, 0]

        nonzero = self._binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        window_img = np.zeros_like(out_img)
        # Generate a polygon to illustrate the search window area
        # And recast the x and y points into usable format for cv2.fillPoly()
        line_window1 = np.array(
            [np.transpose(np.vstack([fitx-margin, ploty]))])
        line_window2 = np.array([np.flipud(np.transpose(np.vstack([fitx+margin,
                                                                   ploty])))])
        line_pts = np.hstack((line_window1, line_window2))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(window_img, np.int_([line_pts]), (0, 255, 0))
        line_pts = np.array(
            [np.transpose(np.vstack([fitx, ploty]))], dtype=np.int32)

        cv2.polylines(out_img, line_pts, isClosed=False,
                      color=(0, 255, 255), thickness=3)

        result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
        return result


# These are constants for the timer callback: Should be modified by UI
l_h = 0
l_s = 0
l_v = 200

u_h = 255
u_s = 50
u_v = 255

lower = np.array([l_h, l_s, l_v])
upper = np.array([u_h, u_s, u_v])

# Coordinates for the 4 alignment points: again, should be handled by the UI
bl = (12, 355)
tl = (66, 304)
br = (635, 344)
tr = (595, 308)
# Aplying perspective transformation
pts1 = np.float32([tl, bl, tr, br])
pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
# Matrix to warp the image for birdseye window
matrix = cv2.getPerspectiveTransform(pts1, pts2)


class Lane_Follower(Node):
    GUI = True

    def __init__(self):
        super().__init__('lane_detection_node')
        self._tolerance = 0
        self._left_follower = Individual_Follower()
        self._right_follower = Individual_Follower()
        # Determine which lane we're in: Left lane means the right image is dashed
        self._Left_Lane = False


        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Publisher for error from the lane lines relative to the camera FOV
        self.camData_publisher = self.create_publisher(
            Float64, '/cam_data', 10)
        self.lane_pubs = self.create_publisher(String, "lane_state",10)
        if Lane_Follower.GUI:
            self._bridge = CvBridge()
            image_labels = ("raw_left", "raw_right", "tf_left",
                            "tf_right", "sliding_left", "sliding_right")
            self._publishers = {label: self.create_publisher(
                Image, "/" + label, 10) for label in image_labels}

    def img_publish(self, label, img_raw):
        if (self.GUI):
            new_img = cv2.cvtColor(img_raw, cv2.COLOR_BGR2RGB)
            self._publishers[label].publish(
                self._bridge.cv2_to_imgmsg(new_img, encoding="passthrough"))

    def measure_position_meters(self, left, right):
        left_x_pos = 0
        right_x_pos = 0

        # Will be the same for left side & right side
        y_max = self._left_follower._binary_warped.shape[0]

        # Calculate left and right line positions at the bottom of the image
        if left is not None:
            left_fit = self._left_follower._fit
            left_x_pos = left_fit[0]*y_max**2 + \
                left_fit[1]*y_max + left_fit[2]
            self.img_publish("sliding_left", left)

        if right is not None:
            right_fit = self._right_follower._fit
            right_x_pos = right_fit[0]*y_max**2 + \
                right_fit[1]*y_max + right_fit[2]
            self.img_publish("sliding_right", right)

        center_lanes_x_pos = (left_x_pos + right_x_pos)//2
        # Calculate the deviation between the center of the lane and the center of the picture
        # The car is assumed to be placed in the center of the picture
        # If the deviation is negative, the car is on the left hand side of the center of the lane
        veh_pos = (
            (self._left_follower._binary_warped.shape[1]//2) - center_lanes_x_pos) * xm_per_pix

        if (left is None):
            veh_pos += 91
        elif (right is None):
            veh_pos -= 91
        return veh_pos / 100

    def determine_lane_size(self, img):
        # Taking in both warped images, determine which lane line is the longer one, and ergo the "solid line"
        # This may struggle on turns, but might work depending: Will need to characterize
        # TODO, parametrize all constants here for tweaking in the U.I
        edges = cv2.Canny(img, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100,
                                minLineLength=100, maxLineGap=5)
        m_length = 0
        if(lines is not None):        
            for line in lines:
                x1, y1, x2, y2 = line[0]
                length = (x1 - x2) ^ 2 + (y1-y2) ^ 2
                m_length = max(m_length, length)
        return m_length

    def timer_callback(self):
        success_l, image_l = vidcap_left.read()
        success_r, image_r = vidcap_right.read()
        images = [(image_l, "left"), (image_r, "right")]
        left_buffer = -1
        right_buffer = -1
        if not (success_l and success_r):
            return

        for image in images:
            frame = image[0]
            # frame = cv2.resize(image[0], (640, 480))
            # I might've cooked with this list comprehension
            for point in (bl, tl, br, tr):
                frame = cv2.circle(frame, point, 5, (0, 0, 255), -1)

            transformed_frame = cv2.warpPerspective(
                frame, matrix, (640, 480))
            # Object Detection
            # Image Thresholding
            hsv_transformed_frame = cv2.cvtColor(
                transformed_frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_transformed_frame, lower, upper)
            if image[1] == "left":
                self._left_follower.set_binwarp(binwarp=mask)
                left_buffer = self.determine_lane_size(mask)
            else:
                self._right_follower.set_binwarp(binwarp=mask)
                right_buffer = self.determine_lane_size(mask)
            self.img_publish("raw_" + image[1], frame)
            self.img_publish("tf_" + image[1], transformed_frame)

        result_left = self._left_follower.Plot_Line()
        result_right = self._right_follower.Plot_Line()
        msg_out = Float64()
        # TODO: Is this the behavior we want? Or do we need it to do something else if one of the lines is invalid?
        if (result_left is not None or result_right is not None):
            pos = self.measure_position_meters(result_left, result_right)
            print(pos)
            msg_out.data = pos
            self.camData_publisher.publish(msg_out)
            msg = String()
            if(left_buffer > right_buffer):
                msg.data = "In Left lane"
                self.lane_pubs.publish(msg)
                self._Left_Lane = True
            elif (right_buffer > left_buffer):
                msg.data = "In Right lane"
                self.lane_pubs.publish(msg)
                self._Left_Lane = False
        else:
            TOLERANCE = 100
            self._tolerance += 1
            if (self._tolerance > TOLERANCE):
                msg_out.data = 1000.0
                self.camData_publisher.publish(msg_out)

        if cv2.waitKey(10) == 27:
            return


def main(args=None):
    rclpy.init(args=args)  # Initialize ROS2 program
    node = Lane_Follower()  # Instantiate Node
    rclpy.spin(node)  # Puts the node in an infinite loop
    # Clean shutdown should be at the end of every node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
