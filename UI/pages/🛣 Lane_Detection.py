import streamlit as st
import cv2

def render_imgs(src, tabs):
    # SRC is cv2.VideoCapture FOR NOW: Will need to be tweaked in the future to rossify
    vidcap = cv2.VideoCapture(src)
    frame_containers = [tab.empty() for tab in tabs]
    while vidcap.isOpened():
        ret, frame = vidcap.read()
        if not ret:
            st.write("Video Capture Stopped")
        else:
            frame_containers[0].image(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB),channels="RGB")
            frame_containers[1].image(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL),channels="HSV")
            frame_containers[2].image(cv2.resize(frame, (144, 144)),channels="BGR")
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    vidcap.release()
    cv2.destroyAllWindows()


## See ../../ros2/src/lanefollowingtest/LaneDetection.py
## This page should render ALL images in there, as well as publish 
if __name__ == "__main__":
    st.set_page_config(
        page_title="Lane Detection",
        page_icon="🛣")
    st.write("This should render all of the images related to Lane Detection, and relevant parameters.")
    st.checkbox("Display Parameters")
    tabs = st.tabs(["Original","HSV", "Sliding Windows"])
    render_imgs("/dev/video0", (tabs))


    ##I Don't even know if we'll want this, but it's a nice to have anyway
    Restart = st.button("ESTOP",type="primary")