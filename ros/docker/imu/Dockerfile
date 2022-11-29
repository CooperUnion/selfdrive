#how to switch from noetic to indigo?? send help

FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update && apt install -y git
RUN apt-get update
RUN apt-get install ros-noetic-serial

# clone ros repo
RUN mkdir -p ros_ws/src

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    rosdep install --from-paths src --ignore-src -r -y  &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release


# copy alias bash and add to bashrc
COPY ./alias.sh /
RUN echo "source /alias.sh" >> ~/.bashrc

CMD ["bash"]
