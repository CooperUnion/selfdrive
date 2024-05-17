import streamlit as st
import cv2
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import time
from std_msgs.msg import String

## This class currently subscribes to some topics in cam_publisher, which I used to test. Please rewrite for lane Detection.
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
            lambda msg: self.camData_callback(msg,(self._frame_containers[0][0], self._frame_containers[0][1])),
            qos_profile_sensor_data)

        self.hsvData_subscriber = self.create_subscription(
            Image,
            '/camera/image_hsv',
            lambda msg: self.camData_callback(msg,(self._frame_containers[1][0], self._frame_containers[1][1])),
            qos_profile_sensor_data)

    def tabs(self, tabs):
        self._tabs = tabs
        self._frame_containers = [(tab.empty(),tab.empty()) for tab in self._tabs]

    #self._frame_containers[] is a container corresponding to one of the tabs: You can create
    def camData_callback(self, msg_in,args):
        raw = msg_in
        img = self._bridge.imgmsg_to_cv2(raw)
        args[0].image(img)
        args[1].image(img)


#This class is your publisher: Flesh it out and integrate accordingly
class Publisher(Node):
    def __init__(self):
            super().__init__('Streamlit_Button_Publisher')
            self.logger = self.get_logger()
            self.button_pub = self.create_publisher(String, '/streamlit/button',10)

def demo_publish():
    msg = String()
    msg.data = "Streamlit Button!"
    st.session_state["pub_node"].button_pub.publish(msg)
    st.success("Message Published!")




# See ../../ros2/src/lanefollowingtest/LaneDetection.py
# This page should render ALL images in there, as well as publish important update data
if __name__ == "__main__":
    #How we Configure the page
    st.set_page_config(
        page_title="Lane Detection",
        page_icon="🛣")
    time.sleep(0.2)

    render = st.checkbox("Render Video Feed")
    st.write(
        "This should render all of the images related to Lane Detection, and relevant parameters.")
    tabs = st.tabs(["Original", "HSV", "Sliding Windows"])

    #This hunk initializes the ROS2 nodes without breaking anything :)
    #Should not need to be tuoched
    if "sub_node" not in st.session_state:
        try:
            rclpy.init()
        except RuntimeError:
            st.warning(
                "something went wrong performance may be degraded. Try restarting fully.")
        finally:
            handler = Image_Handler()
            handler.tabs(tabs)
            st.session_state["sub_node"] = handler
    if "pub_node" not in st.session_state:            
            st.session_state["pub_node"] = Publisher()

#This should also not need to be modified
    if render:
        while (True):
            try:                
                rclpy.spin_once(st.session_state['sub_node'])
                time.sleep(0.01)
            except:
                st.warning(
                    "Something went wrong, perhaps tabs were clicked too quickly? Try restarting.")
                break
    else:
            st.button("Ros Topic Publisher Demo!",on_click=demo_publish)

