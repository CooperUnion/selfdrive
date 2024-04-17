import streamlit as st
import cv2
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import time
from std_msgs.msg import String

# This class currently subscribes to some topics in cam_publisher, which I used to test. Please rewrite for lane Detection.


class Image_Handler(Node):

    def __init__(self, containers):
        self.topics = ["/raw_left", "/raw_right", "/tf_left",
                       "/tf_right", "/sliding_left", "/sliding_right"]
        super().__init__('Streamlit_Image_Handler')
        self._bridge = CvBridge()
        self._left_subscriber = self.create_subscription(Image, self.topics[0], lambda msg: self.camData_callback(msg,containers[0]),qos_profile_sensor_data)
        self._right_subscriber = self.create_subscription(Image, self.topics[1], lambda msg: self.camData_callback(msg,containers[1]),qos_profile_sensor_data)
        self._tf_left_subscriber = self.create_subscription(Image, self.topics[2], lambda msg: self.camData_callback(msg,containers[2]),qos_profile_sensor_data)
        self._tf_right_subscriber = self.create_subscription(Image, self.topics[3], lambda msg: self.camData_callback(msg,containers[3]),qos_profile_sensor_data)
        self._sliding_left_subscriber = self.create_subscription(Image, self.topics[4], lambda msg: self.camData_callback(msg,containers[4]),qos_profile_sensor_data)
        self._sliding_right_subscriber = self.create_subscription(Image, self.topics[5], lambda msg: self.camData_callback(msg,containers[5]),qos_profile_sensor_data)


    # self._frame_containers[] is a container corresponding to one of the tabs: You can create
    def camData_callback(self, msg_in, img_location):
        raw = msg_in
        img = self._bridge.imgmsg_to_cv2(raw, desired_encoding="passthrough")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        with img_location:
            st.image(img)


# This class is your publisher: Flesh it out and integrate accordingly
class Publisher(Node):
    def __init__(self):
        super().__init__('Streamlit_Button_Publisher')
        self.logger = self.get_logger()
        self.button_pub = self.create_publisher(
            String, '/streamlit/button', 10)


def demo_publish():
    msg = String()
    msg.data = "Streamlit Button!"
    st.session_state["pub_node"].button_pub.publish(msg)
    st.success("Message Published!")


# See ../../ros2/src/lanefollowingtest/LaneDetection.py
# This page should render ALL images in there, as well as publish important update data
if __name__ == "__main__":
    # How we Configure the page
    st.set_page_config(
        page_title="Lane Detection",
        page_icon="🛣")
    time.sleep(0.2)

    render = st.checkbox("Render Video Feed")
    st.write(
        "This should render all of the images related to Lane Detection, and relevant parameters.")
    #Creates the Structure
    tabs = st.tabs(["Original", "Birds Eye View", "Sliding Windows"])
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
    if "sub_node" not in st.session_state:
        try:
            rclpy.init()
        except RuntimeError:
            st.warning(
                "something went wrong performance may be degraded. Try restarting fully.")
        finally:
            handler = Image_Handler(frame_containers)
            st.session_state["sub_node"] = handler
    if "pub_node" not in st.session_state:
        st.session_state["pub_node"] = Publisher()

# This should also not need to be modified
    if render:
        while (True):
            # try:
            rclpy.spin_once(st.session_state['sub_node'])
            time.sleep(.01)
            # except:
            #     st.warning(
            #         "Something went wrong, perhaps tabs were clicked too quickly? Try restarting.")
            #     break
    else:
        st.button("Ros Topic Publisher Demo!", on_click=demo_publish)
