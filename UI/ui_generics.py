import streamlit as st

def sidebar():
    with st.sidebar:
        st.page_link("Homepage.py", label=":derelict_house_building: Homepage", icon=None, help="Opens Homepage", disabled=False, use_container_width=True)
        st.page_link("pages/Lane_Detection.py", label=":motorway: Lane Detection", icon=None, help="Opens Lane detection", disabled=False, use_container_width=True)
        st.page_link("pages/Object_Detection.py", label=":octagonal_sign: Object Detection", icon=None, help="Opens Object detection", disabled=False, use_container_width=True)