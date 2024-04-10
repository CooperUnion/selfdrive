import streamlit as st
import cv2
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import time


class Image_Handler(Node):

    def __init__(self):
        self._tabs = None
        self._img = None
        self._frame_containers = None
        super().__init__('Streamlit_Image_Handler')
        self._bridge = CvBridge()
        self.imgData_subscriber = self.create_subscription(
            Image,
            '/camera/image',
            self.camData_callback,
            qos_profile_sensor_data)
        self.hsvData_subscriber = self.create_subscription(
            Image,
            '/camera/image_hsv',
            self.hsvData_callback,
            qos_profile_sensor_data)


    def tabs(self,tabs):
        self._tabs = tabs
        # SRC is cv2.VideoCapture FOR NOW: Will need to be tweaked in the future to rossify
        self._frame_containers = [tab.empty() for tab in self._tabs]


    def camData_callback(self, msg_in):
        raw = msg_in
        img = self._bridge.imgmsg_to_cv2(raw)
        self._frame_containers[0].image(img)
     
    def hsvData_callback(self, msg_in):
        raw = msg_in
        img = self._bridge.imgmsg_to_cv2(raw)
        self._frame_containers[1].image(img)

     
# See ../../ros2/src/lanefollowingtest/LaneDetection.py
# This page should render ALL images in there, as well as publish
if __name__ == "__main__":
    st.set_page_config(
        page_title="Lane Detection",
        page_icon="🛣")
    time.sleep(0.2)

    render = st.checkbox("Render Video Feed")
    st.write(
        "This should render all of the images related to Lane Detection, and relevant parameters.")
    tabs = st.tabs(["Original", "HSV", "Sliding Windows"])

    if "node" not in st.session_state:
            try:
                rclpy.init()
            except RuntimeError:
                 st.warning("something went wrong performance may be degraded. Try restarting fully")
            finally:                 
                handler = Image_Handler()
                handler.tabs(tabs)
                st.session_state["node"] = handler
    if render:
            while(True):
                try:
                    rclpy.spin_once(st.session_state['node'])
                    time.sleep(0.01)                     
                except:
                     st.warning("Something went wrong, perhaps tabs were clicked too quickly? Try restarting.")
                     break
