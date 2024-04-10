import streamlit as st
from streamlit import runtime
from streamlit.web import cli as stcli
import sys


def runner():
    st.header("Welcome to the IGVC Homepage!")
    st.markdown('''
             Please select an option on the sidebar to the right: This specifies which UI file you'd like to run   


             Streamlit supports fast reloads: Just refresh the python script to get going!


             Docs are here: https://docs.streamlit.io/library/api-reference
             ''')
    st.divider()
    st.markdown('''
             For the freshmen: This UI is designed to serve multiple purposes 
             
             #1. We need a way to render all of the openCV data we have: This is stuff like the lane or object detection.   

             #2. We need a way to tune  parameters in an easy way, without refreshing every single page for small little tweaks.    

             #3. We need to be able to control the state machines on the car: Each of the function tests is effectively it's own state machine.
             We need to be able to control which test we're in, and visualize any important data in a direct manner. 
                
               
             #4.  It makes us look professional, and adds quite a bit of useful fluff for our design report.
             ''')
    st.divider()
    st.markdown('''
    Of course, not a lot of this is in your control: We haven't built the state machines or finished all of our algorithms.   
                However, Lane Detection is in the perfect place for you to familiarize yourself with both streamlit and the entire tech stack.   
                I'm detailing all immediate tasks here: Please allocate amongst yourselves.   
                I have taken the liberty of building a bespoke ros2 publisher and subscriber system (was not trivial), that should provide a solid framework.   
                However, it will get extremely tedious to build a proper UI if you don't exploit some of the advantages of python.   
                Read about list comprehension and get good.    
                #1: Connecting all datastreams from the lane_detection algorithm into here.
                Everything that is cv.imshow() is rendered in that, in a clean and effective manner.      
                #2: Parametrizing all constants in that: anything that had a slider should be here, and it should be published.   
                #3: Beautifying, and building more tabs & systems so this is the only interface we need with the car. 
                We should not be opening up or handling multiple windows with the judges.
                Debugging systems, sensor messages (E.g IMU, CAN) should all live here.   

                ''')
    st.warning("FIRST PRIORITY: https://docs.streamlit.io/develop/api-reference/widgets/st.page_link CHANGE THE PAGES SO NO MORE EMOJIS")


# This allows the system to run without being in a dedicated streamlit session:
if __name__ == "__main__":
    if runtime.exists():
        st.set_page_config(
            page_title="IGVC Homepage",
            page_icon="🚗")
        st.sidebar.success("Select a System above.")
        runner()
    else:
        sys.argv = ["streamlit", "run", sys.argv[0]]
        sys.exit(stcli.main())
