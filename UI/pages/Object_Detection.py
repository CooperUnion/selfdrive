import streamlit as st
from ui_generics import *


# This class currently subscribes to some topics in cam_publisher, which I used to test. Please rewrite for Object Detection.


if __name__ == "__main__":
    st.set_page_config(
        page_title="Object Detection",
        page_icon="🛑")
    sidebar()