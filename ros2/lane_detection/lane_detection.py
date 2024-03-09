import rclpy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node # Node class is the is the base class all ROS2 nodes are derived from


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')  # Initializes ROS2 node called 'turtlebot4_first_python_node'
        
        self.bridge = CvBridge()
        # Subscribe to the /interface_buttons topic
        self.command_subscriber = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            qos_profile=1)

    def image_callback(self, msg):
        print("callbacked!")       
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        detect(cv_image)


def getLargestLine(frame: np.ndarray):
    #https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html 
    edge_image = cv.Canny(frame, 250, 150)
    lines = cv.HoughLinesP(
        edge_image, 1, np.pi / 180, 100, minLineLength=30, maxLineGap=250)
    maxlen = 0
    #Guarantees that even if theres no line, we're setting it to the maximum potential value,
    ##so there's no functional error
    maxlendata = [frame.shape[1], 0, 0, 0]
    #Iterate over all found lines
    if lines is not None:
        for l in lines[:]:
            cur = l[0]
            #Don't need to take the root of this: quantized means all values are positive
            #Len2 as it's len squared
            if len2 := (cur[2] - cur[0])**2 + (cur[3] - cur[1])**2 > maxlen:
                #Walrus because why not
                maxlen = len2
                maxlendata = cur
    return maxlendata



def detect(image: np.ndarray):
        #Handles lane detection and error calculation
        width = image.shape[1] // 2
        # Splitting images here as we only have one source: In other case, this code would be same for both cameras
        # and a rosnode would compare outputs
        frames = [image[:, :width], image[:, width:]]
        # Need this to persist
        errors = []
        for ind in range(0,len(frames)):
            # Convert to hsv, perform basic image processing on HSV ALONE
            hsv = cv.cvtColor(frames[ind], cv.COLOR_BGR2HSV_FULL)
            gray = cv.GaussianBlur(hsv[:, :, 2], (9, 9), 0)
            threshed = cv.threshold(gray, 200, 255, cv.THRESH_BINARY_INV)[1]

            # Get the largest lane line, on processed image, this is what will be used
            line = getLargestLine(threshed)

            # Draw the line
            cv.line(
                image,
                (line[0], width*ind+line[1]),
                (line[2], width*ind+line[3]),
                (0, 0, 255),
                1,
                cv.LINE_AA,
            )
            # Writing the "error": In essence, our distance to the line .
            # This uses the average x coordinate as a reference to where the line "sits", and should not be swayed by warping.
            errors.append(abs(line[0] - line[2]) / 2)
        # This should be input to controller
        net_error = errors[0] - errors[1]
        print(net_error)
        # Necessary for cv to work, otherwise it gets scared
        cv.imshow("Source with lines ",image)
        cv.waitKey(1)

def main(args=None):
    rclpy.init(args=args) # Initialize ROS2 program
    node = LaneDetectionNode() # Instantiate Node
    rclpy.spin(node) # Puts the node in an infinite loop
    
    # Clean shutdown should be at the end of every node
    node.destroy_node() 
    cv.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()