FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update && apt install -y git gdb xterm vim

RUN apt-get -y update

RUN apt-get install -y ros-noetic-map-server ros-noetic-move-base ros-noetic-rviz

# clone ros repo
RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/ros-planning/navigation.git ros_ws/src/navigation &&\
    git clone https://github.com/rst-tu-dortmund/teb_local_planner.git ros_ws/src/teb_local_planner &&\
    git clone https://github.com/GMahmoud/custom_msgs_demo.git ros_ws/src/custom_msgs

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\ 
    apt update --fix-missing &&\
    rosdep install --from-paths src --ignore-src -r -y &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

RUN git clone https://github.com/GMahmoud/virtual_costmap_layer.git ros_ws/src/virtual_costmap_layer 
# modify CMake file
# change line 3 to 'file(STRINGS ${CMAKE_SOURCE_DIR}/virtual_costmap_layer/VERSION version)'
# rerun catkin_make

COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc
RUN echo "source /entrypoint.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
