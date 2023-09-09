FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /app

# install dep and such
RUN apt update && apt install -y git ros-noetic-velodyne

# clone ros repo
RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/ros-drivers/velodyne ros_ws/src/velodyne

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    apt update --fix-missing &&\
    rosdep install --from-paths src --ignore-src -r -y  &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

# copy alias bash and add to bashrc
COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc
RUN echo "source /entrypoint.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
