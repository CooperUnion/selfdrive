# FROM ros:noetic-ros-base
FROM gazebo:libgazebo11

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

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

# # ------------------------
# # --- GAZEBO SERVER 11 ---
# # ------------------------
# # setup timezone
# RUN apt-get update && \
#     apt-get install -q -y --no-install-recommends tzdata apt-utils

# # install packages
# RUN apt-get update && apt-get install -q -y --no-install-recommends \
#     dirmngr \
#     gnupg2 \
#     lsb-release

# # setup keys
# RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# # setup sources.list
# RUN . /etc/os-release \
#     && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# # install gazebo packages
# RUN apt-get update && apt-get install -y --no-install-recommends \
#     gazebo11=11.9.0-1*

# # setup environment
# EXPOSE 11345

RUN apt update && apt install -y git

RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git ros_ws/src/gazebo-packages && \
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git ros_ws/src/turtle

RUN cd ros_ws && \
    source /opt/ros/noetic/setup.bash && \
    rosdep fix-permissions && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -y && \
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release && \
    export TURTLEBOT3_MODEL=burger

COPY ./entrypoint.sh /

ENV TURTLEBOT3_MODEL='waffle_pi'

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
