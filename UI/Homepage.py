import streamlit as st
from streamlit import runtime
from streamlit.web import cli as stcli
import sys

def runner():
    st.header("Welcome to the IGVC Homepage!")
    st.write("Please select an option on the sidebar to the right.")
    st.write("For the freshmen: You guys can beautify this up however you'd like")
    st.write(
        "Streamlit supports fast reloads, just refresh the python script to get going!")
    st.write("Docs are here: https://docs.streamlit.io/library/api-reference")
    st.write("This is also extremely relevant: https://stackoverflow.com/questions/67382181/how-to-use-streamlit-in-ros")



# This allows the system to run without being in a dedicated streamlit session:
if __name__ == "__main__":
    if runtime.exists():
        st.set_page_config(
            page_title="IGVC Homepage",
            page_icon="🚗")
        st.sidebar.success("Select a demo above.")
        runner()
    else:
        sys.argv = ["streamlit", "run", sys.argv[0]]
        sys.exit(stcli.main())
