FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update && apt install -y git ros-noetic-velodyne

# clone ros repo
RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/ros-drivers/velodyne ros_ws/src/velodyne

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]