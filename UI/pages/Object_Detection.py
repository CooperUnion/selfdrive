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
import subprocess
from subprocess import call
import sys
from geometry_msgs.msg import Transform
import os


# This class currently subscribes to some topics in cam_publisher, which I used to test. Please rewrite for lane Detection.


if __name__ == "__main__":
    st.set_page_config(
        page_title="Lane Detection",
        page_icon="🛑")
    sidebar()

if 'button' not in st.session_state:
    st.session_state.button = False


def click_button():
    st.session_state.button = not st.session_state.button


st.button('Object Detection', type="primary", on_click=click_button)

if st.session_state.button:
    st.write("working")
    os.chdir('../../Yolo-World-Detection/dist_to_obj_ws')
    call('colcon build', shell=True)
    call('ls -a', shell=True)
    # making it wait just in case it requires it to make the package
    time.sleep(0.05)
    call("source install/setup.bash", shell=True)
    time.sleep(0.05)  # same as above
    call("ros2 run dist_to_obj_package dist_to_obj_node", shell=True)
