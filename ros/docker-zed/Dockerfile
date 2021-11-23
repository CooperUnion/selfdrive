# https://www.stereolabs.com/docs/docker/install-guide-linux/
# https://gitlab.com/nvidia/container-images/cuda/blob/master/dist/11.4.2/ubuntu2004/base/Dockerfile

FROM stereolabs/zed:3.6-gl-devel-cuda11.4-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# ------------------------
# ------- ROS CORE -------
# ------------------------
# setup timezone
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata apt-utils

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1*

# ------------------------
# ------- ROS BASE -------
# ------------------------
# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* 

WORKDIR /app

RUN apt update && apt install -y git 

RUN mkdir -p ros_ws/src &&\
    git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git ros_ws/src/zed && \
    git clone --recursive https://github.com/stereolabs/zed-ros-examples.git ros_ws/src/zed-examples

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
