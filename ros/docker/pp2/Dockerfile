FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update && apt install -y git

# RUN cd /usr/include &&\
    
# clone ros repo

# RUN mkdir -p ros_ws/src &&\
#     git clone https://github.com/orocos/orocos_kinematics_dynamics.git

# RUN cd ros_ws &&\
#     source /opt/ros/noetic/setup.bash &&\ 
#     catkin_make_isolated

RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/ros-drivers/ackermann_msgs.git &&\
    git clone https://github.com/f1tenth-dev/pure_pursuit.git ros_ws/src/pure_pursuit &&\
    git clone https://github.com/orocos/orocos_kinematics_dynamics.git

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\ 
    rosdep install --from-paths src --ignore-src -r -y  &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

