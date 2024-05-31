import streamlit as st
from streamlit import runtime
from streamlit.web import cli as stcli
import sys
from ui_generics import *
import threading


class Image_Handler:
    """Handles Asynchronously running the image updates, removing blocking"""

    def __init__(self):
        self.checkbox = True
        self.thread: threading.Thread

    def __enter__(self):
        """ Updates the image """
        self.checkbox = st.checkbox("Render State Machine")
        @with_streamlit_context
        def update_image() -> None:
            """
            Update the progress bar asynchronously.
            """
            container = st.empty()
            while self.checkbox:
                container.image("/home/eeadmin/selfdrive/live_SM.png")

        self.thread = threading.Thread(target=update_image)
        self.thread.start()
        return self

    def __exit__(self, *args) -> None:
        """
        Stop the progress bar when you exit the `with` context.
        """
        self.stop()

    def stop(self) -> None:
        """
        Stop the progress bar.
        """
        if self.thread.is_alive():
            self.running = False
            self.thread.join()


def runner():

    st.header("Welcome to the IGVC Homepage!")
    st.markdown('''
             Please select an option on the sidebar to the right: This specifies which UI file you'd like to run   


             Streamlit supports fast reloads: Just refresh the python script to get going!


             Docs are here: https://docs.streamlit.io/library/api-reference
             ''')
    st.divider()
    with Image_Handler():
        pass

if __name__ == "__main__":
    if runtime.exists():
        st.set_page_config(
            page_title="IGVC Homepage",
            page_icon="🚗")
        sidebar()
        runner()
    else:
        sys.argv = ["streamlit", "run",
                    "--client.showSidebarNavigation=False", sys.argv[0]]
        sys.exit(stcli.main())
