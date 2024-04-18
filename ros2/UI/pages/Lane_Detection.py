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

# This class currently subscribes to some topics in cam_publisher, which I used to test. Please rewrite for lane Detection.


class Image_Handler(Node):

    def __init__(self, containers):
        self.reference_img = None
        self.topics = ["/camera/image", "/raw_right", "/tf_left",
                       "/tf_right", "/sliding_left", "/sliding_right"]
        super().__init__('Streamlit_Image_Handler')
        self._bridge = CvBridge()
        self.containers = containers
        self.lane_state_pub = self.create_subscription(
            String, "lane_state", self.lane_state_callback, qos_profile_sensor_data)
        
        self._left_subscriber = self.create_subscription(
            Image, self.topics[0], lambda msg: self.camData_callback(msg, self.containers[0], img_to_repl="THIS"), qos_profile_sensor_data)
        self._right_subscriber = self.create_subscription(
            Image, self.topics[1], lambda msg: self.camData_callback(msg, self.containers[1]), qos_profile_sensor_data)
        self._tf_left_subscriber = self.create_subscription(
            Image, self.topics[2], lambda msg: self.camData_callback(msg, self.containers[2]), qos_profile_sensor_data)
        self._tf_right_subscriber = self.create_subscription(
            Image, self.topics[3], lambda msg: self.camData_callback(msg, self.containers[3]), qos_profile_sensor_data)
        self._sliding_left_subscriber = self.create_subscription(
            Image, self.topics[4], lambda msg: self.camData_callback(msg, self.containers[4]), qos_profile_sensor_data)
        self._sliding_right_subscriber = self.create_subscription(
            Image, self.topics[5], lambda msg: self.camData_callback(msg, self.containers[5]), qos_profile_sensor_data)

    # self._frame_containers[] is a container corresponding to one of the tabs: You can create

    def camData_callback(self, msg_in, img_location, img_to_repl=None):
        raw = msg_in
        img = self._bridge.imgmsg_to_cv2(raw, desired_encoding="passthrough")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        if (img_to_repl is not None):
            self.reference_img = img
        with img_location:
            st.image(img)

    def lane_state_callback(self, msg):
        st.write(msg.data)


# This class is your publisher: Flesh it out and integrate accordingly
class Publisher(Node):
    def __init__(self):
        super().__init__('Streamlit_Button_Publisher')
        self.logger = self.get_logger()
        self.button_pub = self.create_publisher(
            String, '/streamlit/button', 10)


def render_handler(context):
    rclpy.try_shutdown()
    rclpy.init()
    context.empty()
    with context:
        tabs = st.tabs(
            ["Original", "Birds Eye View", "Sliding Windows"])
        containers = []
        frame_containers = []
        for tab in tabs:
            cols = tab.columns(2)
            containers.append(cols[0])
            containers.append(cols[1])
            with cols[0]:
                frame_containers.append(st.empty())
            with cols[1]:
                frame_containers.append(st.empty())
        # This hunk initializes the ROS2 nodes without breaking anything :)
        # Should not need to be tuoched
        st.write(len(frame_containers))
        handler = get_publisher(frame_containers)
        while (True):
            # try:
                rclpy.spin_once(handler)
                time.sleep(.01)
            # except:
            #     with st.empty():
            #         st.warning("Something went wrong, unrender & rerender, close all other tabs.")
            #         break
def demo_publish():
    publisher = get_publisher()
    msg = String()
    msg.data = "Streamlit Button!"
    publisher.button_pub.publish(msg)
    st.success("Message Published!", icon="✅")


def get_from_prior(labels, defaults):
    defaults = np.resize(defaults, len(labels))
    for i in range(len(labels)):
        if labels[i] not in st.session_state:
            st.session_state[labels[i]] = defaults[i]
        else:
            defaults[i] = st.session_state[labels[i]]
    return defaults


def input_gen(func, labels, lowers, uppers, vals):
    lowers = np.resize(lowers, len(labels))
    uppers = np.resize(uppers, len(labels))
    func_list = []
    for i in range(len(labels)):
        func_list.append(func(
            labels[i], lowers[i], uppers[i], vals[i]))
    return func_list


def input_handler(context):
    rclpy.try_shutdown()
    rclpy.init()
    with context:
        # handles all output
        # We use get_from_prior to ensure that, even if the render checkbox was flipped, data persists
        # Input gen allows me to streamline generating a lot of objects by making it super duper quick
        tabs = st.tabs(["HSV Tweakers", "Coordinates", "Misc"])
        # HSV user input
        with tabs[0]:
            l, h = st.columns(2)
            # lower HSV slider
            L_labels = ["Lower Hue", "Lower Saturation", "Lower Value"]
            default_vals = [0, 0, 200]
            default_vals = get_from_prior(L_labels, default_vals)

            l_h, l_s, l_v = input_gen(l.slider, L_labels, [0], [255], default_vals)
            U_labels = ["Upper Hue", "Upper Saturation", "Upper Value"]
            default_vals = [255, 50, 255]
            default_vals = get_from_prior(U_labels, default_vals)
            # upper HSV slider
            u_h, u_s, u_v = input_gen(h.slider, U_labels, [0], [255], default_vals)
        # Coordinate input widget
        with tabs[1]:
            x, y = st.columns(2)
            x_labels = ["Bottom Left x", "Top Left x",
                        "Bottom Right x", "Top Right x"]
            default_vals = [12, 66, 635, 595]
            default_vals = get_from_prior(x_labels, default_vals)
            bl_x, tl_x, br_x, tr_x = input_gen(
                x.number_input, x_labels, [0], [640], default_vals)
            y_labels = ["Bottom Left y", "Top Left y",
                        "Bottom Right y", "Top Right y"]
            default_vals = [355, 304, 344, 308]
            default_vals = get_from_prior(y_labels, default_vals)
            bl_y,  tl_y, br_y, tr_y = input_gen(
                y.number_input, y_labels, [0], [480], default_vals)

        # This is souced from LaneDetection
        bl = (bl_x, bl_y)
        tl = (tl_x, tl_y)
        br = (br_x, br_y)
        tr = (tr_x, tr_y)
        pts1 = np.float32([tl, bl, tr, br])
        pts2 = np.float32([[0, 0], [0, 480], [640, 0], [640, 480]])
        # Matrix to warp the image for birdseye window

        # This is what we're really looking for, and what we might want to consider returning as a custom ros message...
        # https://github.com/Box-Robotics/ros2_numpy
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        lower = np.array([l_h, l_s, l_v])
        upper = np.array([u_h, u_s, u_v])

        pub = get_publisher(None)
        img = pub.reference_img

        if img is not None:
            cols = st.columns(3)
            cols[0].image(img, "Left reference image for transformation")
            transformed_left = cv2.warpPerspective(img, matrix, (640, 480))
            hsv_transformed_left = cv2.cvtColor(
                transformed_left, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_transformed_left, lower, upper)
            cols[1].image(transformed_left, "Left Birds Eye View Preview")
            cols[2].image(mask, "Left Mask Preview")
    # TODO: Make this button publish the custom ROS2 Message :)
        st.button("Publish!", on_click=demo_publish)
        #TODO: Make this ubtton do soemthign
        st.button("RESET TO DEFAULT")

@st.cache_resource 
def get_publisher(_tabs):
    handler = Image_Handler(_tabs)
    return handler

@st.cache_resource
def get_subscriber():
    handler = Publisher()
    return handler
    


# See ../../ros2/src/lanefollowingtest/LaneDetection.py
# This page should render ALL images in there, as well as publish important update data
if __name__ == "__main__":
    # How we Configure the page
    st.set_page_config(
        page_title="Lane Detection",
        page_icon="🛣")
    sidebar()

    time.sleep(0.1)
    st.write(
        "This page is designed to control the lane detection functionality, and allow for quick edits to the algorithm.")
    render = st.checkbox("Render Video Feed",value=True)
    display_holder = st.container()
    if render:
        render_handler(display_holder)
    else:
        input_handler(display_holder)
