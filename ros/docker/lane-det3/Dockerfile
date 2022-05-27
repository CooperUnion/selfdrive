FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update && apt install -y git &&\
    apt-get install -y ros-noetic-costmap-2d

# clone ros repo
RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/gutsy-robot/lane_costmap_layer.git ros_ws/src/lane_costmap_layer

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    rosdep install --from-paths src --ignore-src -r -y  &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
