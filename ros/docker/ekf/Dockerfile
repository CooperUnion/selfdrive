FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /app

# install dep and such
RUN apt update && apt install -y git 

# clone ros repo
RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/ros-planning/robot_pose_ekf.git ros_ws/src/robot_pose_ekf

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\ 
    apt update --fix-missing &&\
    rosdep install --from-paths src --ignore-src -r -y  &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

# copy alias bash and add to bashrc
COPY ./alias.sh /
RUN echo "source /alias.sh" >> ~/.bashrc

CMD ["bash"]