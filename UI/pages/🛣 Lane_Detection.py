import streamlit as st
import cv2
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

IMG_CATEGORIES = ["raw","tf","sliding"]

class Image_Handler(Node):

    def __init__(self):
        self._tab_dict = None
        self._frame_containers = None
        super().__init__('Streamlit_Image_Handler')
        self._bridge = CvBridge()
        image_labels = ("raw_left", "raw_right", "tf_left", "tf_right", "sliding_left", "sliding_right")
        #Multithread the subscribers to separate rosnodes.
        self._subscribers = (self.create_subscription(Image, "/" + label, lambda msg: self.camData_callback(msg,label),qos_profile_sensor_data) for label in image_labels)


    def tabs(self,tabs):
        self._tab_dict = tabs


    def camData_callback(self, msg_in,label):
        st.write("Put image from %s" % label)
        img = self._bridge.imgmsg_to_cv2(msg_in)
        self.render_imgs(img,label.split('_')[0])

    def render_imgs(self,img,label):
        #Specifies which camera the image is from
        if img is not None:
            tab_dict[label].image(cv2.cvtColor(self._img,cv2.COLOR_BGR2RGB),channels="RGB")




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
    render = st.checkbox("Display Camera Output")
    tabs = st.tabs(IMG_CATEGORIES)        
    imgs_w_places = tuple(zip(tabs, [tab.columns(2) for tab in tabs]))
    #Dictionary 
    tab_dict = dict(zip(IMG_CATEGORIES,imgs_w_places))

    if render:
        ros_node = image_instantiator()
        ros_node.tabs(tab_dict)
        rclpy.spin(ros_node)
        ros_node.destroy_node()
        rclpy.shutdown()


    # I Don't even know if we'll want this, but it's a nice to have anyway
    Restart = st.button("ESTOP", type="primary")


