FROM ubuntu:14.04

ARG DEBIAN_FRONTEND=noninteractive

ENV ROS_DISTRO indigo

RUN apt update && \
    apt install -y curl vim 

# Register ROS repository
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN apt update && \
    apt install -y ros-indigo-ros-base ros-indigo-mrpt-navigation

RUN rosdep init && \
    rosdep update

# Setup ROS environment variables globally
RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc
