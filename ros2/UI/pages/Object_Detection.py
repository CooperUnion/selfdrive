import streamlit as st
import cv2
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import time
from std_msgs.msg import String
import numpy as np
from ui_generics import *
import ros2_numpy as rnp
from geometry_msgs.msg import Transform

# This class currently subscribes to some topics in cam_publisher, which I used to test. Please rewrite for lane Detection.


class Object_Handler(Node):

    def __init__(self, containers):
        self.reference_img = None
        self.topics = ["/obj_detection_img"]
        super().__init__('Streamlit_Object_Detection_Handler')
        self._bridge = CvBridge()
        self.containers = containers
        self.dist_sub = self.create_subscription(
            String, "/dist_to_obj", lambda msg: self.distance_callback(msg,self.containers[1]), qos_profile_sensor_data)

        self._obj_detection_subscriber = self.create_subscription(
            Image, self.topics[0], lambda msg: self.camData_callback(msg, self.containers[0], img_to_repl="THIS"), qos_profile_sensor_data)
    # self._frame_containers[] is a container corresponding to one of the tabs: You can create

    def camData_callback(self, msg_in, img_location, img_to_repl=None):
        raw = msg_in
        img = self._bridge.imgmsg_to_cv2(raw, desired_encoding="passthrough")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        if (img_to_repl is not None):
            self.reference_img = img
        with img_location:
            st.image(img)
    
    def distance_callback(self,msg,container):
        pass
        # container.write(msg)




# This class is your publisher: Flesh it out and integrate accordingly
class Publisher(Node):
    def __init__(self):
        self.data = None
        super().__init__('Streamlit_Button_Publisher')
        self.logger = self.get_logger()
        self.image_pub= self.create_publisher(
        Image, 'streamlit/lane_data', 10)
        self.button_pub = self.create_publisher(
            String, '/streamlit/button', 10) 

def render_handler(context):
    rclpy.try_shutdown()
    rclpy.init()
    context.empty()
    with context:
        tabs = st.tabs(
            ["Object Detection"])
        frame_containers = []
        for tab in tabs:
            frame_containers.append(tab.empty())
        frame_containers.append(st.empty())
        # This hunk initializes the ROS2 nodes without breaking anything :)
        # Should not need to be tuoched
        handler = get_new_subscriber(frame_containers)
        while (True):
            try:
                rclpy.spin_once(handler)
            except:
                rclpy.try_shutdown()
                st.warning(
                    "Generator already spinning. Try flipping the Render Switch")
                break



def input_handler(context):
    pass

@st.cache_resource
def get_new_subscriber(_tabs):
    handler = Object_Handler(_tabs)
    return handler


# See ../../ros2/src/lanefollowingtest/LaneDetection.py
# This page should render ALL images in there, as well as publish important update data
if __name__ == "__main__":
    # How we Configure the page
    st.set_page_config(
        page_title="Object Detection",
        page_icon="🛑")
    sidebar()
    st.write(
        "This page is designed to control the object detection functionality, and allow for quick edits to the algorithm.")
    render = st.checkbox("Render Video Feed", value=False)
    display_holder = st.container()
    if render:
        render_handler(display_holder)