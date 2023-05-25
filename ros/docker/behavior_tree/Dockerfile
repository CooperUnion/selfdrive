FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update --fix-missing && apt install -y git

RUN apt-get -y update --fix-missing 

# clone ros repo
RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/miccol/ROS-Behavior-Tree.git ros_ws/src/behavior_tree

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    rosdep install --from-paths src --ignore-src -r -y &&\
    catkin_make

COPY ./entrypoint.sh /

COPY ./alias.sh /
RUN echo "source /alias.sh" >> ~/.bashrc
RUN echo "source /entrypoint.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]