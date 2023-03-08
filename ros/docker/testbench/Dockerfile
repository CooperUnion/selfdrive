
FROM ros:noetic-ros-base 

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update && apt install -y curl

RUN apt-get -y update

RUN mkdir -p ros_ws/src

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    rosdep install --from-paths src --ignore-src -r -y &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /
COPY ./alias.sh /


RUN echo "source /alias.sh" >> ~/.bashrc

#Trying Something Here For Serial Communication over USB
ENTRYPOINT [ "/entrypoint.sh" ]
CMD ["bash"]
