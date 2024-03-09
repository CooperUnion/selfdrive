
import rclpy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.node import Node # Node class is the is the base class all ROS2 nodes are derived from


class LaneDetectionNode(Node):
    # Run ros2 launch turtlebot4_bringup oakd.launch.py on tbot
    def __init__(self):
        super().__init__('lane_detection_node')  # Initializes ROS2 node called 'turtlebot4_first_python_node'

        self.bridge = CvBridge()
        self.cv_image = None
        # Subscribe to the /interface_buttons topic
        self.command_subscriber = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',
            self.image_callback,
            qos_profile=1)
        
    def image_callback(self, msg):
        print("callbacked!")       
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        cv.imshow('frame',self.cv_image)
        cv.waitKey(1)




def main(args=None):
    rclpy.init(args=args) # Initialize ROS2 program
    node = LaneDetectionNode() # Instantiate Node
    rclpy.spin(node) # Puts the node in an infinite loop
    # Clean shutdown should be at the end of every node
    node.destroy_node() 
    rclpy.shutdown()


if __name__ == '__main__':
    main()