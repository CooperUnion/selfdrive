import streamlit as st
import cv2
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import typing

class Image_Handler(Node):

    def __init__(self):
        self._tabs = None
        self._img = None
        self._frame_containers = None
        super().__init__('Streamlit_Image_Handler')
        self._bridge = CvBridge()
        self.camData_subscriber = self.create_subscription(
            Image,
            '/camera/image',
            self.camData_callback,
            qos_profile_sensor_data)

    def tabs(self,tabs):
        self._tabs = tabs
        # SRC is cv2.VideoCapture FOR NOW: Will need to be tweaked in the future to rossify
        self._frame_containers = [tab.empty() for tab in self._tabs]


    def camData_callback(self, msg_in):
        raw = msg_in
        self._img = self._bridge.imgmsg_to_cv2(raw)
        self.render_imgs()

    def render_imgs(self):
        # SRC is cv2.VideoCapture FOR NOW: Will need to be tweaked in the future to rossify
        if self._img is not None:
            for frame_container in self._frame_containers:
                frame_container.image(cv2.cvtColor(self._img,cv2.COLOR_BGR2RGB),channels="RGB")

@st.cache_resource
def image_instantiator():
    rclpy.init(args=None)
    return Image_Handler()
     


# See ../../ros2/src/lanefollowingtest/LaneDetection.py
# This page should render ALL images in there, as well as publish
if __name__ == "__main__":
    st.set_page_config(
        page_title="Lane Detection",
        page_icon="🛣")
    st.write(
        "This should render all of the images related to Lane Detection, and relevant parameters.")
    render = st.checkbox("Display Parameters")
    tabs = st.tabs(["Original", "HSV", "Sliding Windows"])

    if not render:
        ros_node = image_instantiator()
        ros_node.tabs(tabs)
        rclpy.spin(ros_node)
        ros_node.destroy_node()
        rclpy.shutdown()


    # I Don't even know if we'll want this, but it's a nice to have anyway
    Restart = st.button("ESTOP", type="primary")
