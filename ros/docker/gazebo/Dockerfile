FROM gazebo:gzserver11-focal

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# ------------------------
# ------- ROS CORE -------
# ------------------------
# setup timezone
RUN apt-get update -y && \
    apt-get install -q -y --no-install-recommends tzdata apt-utils

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654



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

###############

WORKDIR /app

# install dep and such
RUN apt update && apt upgrade -y && apt install -y git

RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git ros_ws/src/gazebo-packages &&\
    git clone https://github.com/ros-teleop/teleop_twist_keyboard.git ros_ws/src/teleop_twist

# install silly packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgazebo11-dev=11.12.0-1* \
    ros-noetic-control-toolbox \
    ros-noetic-controller-manager \
    ros-noetic-hardware-interface \
    ros-noetic-transmission-interface \
    ros-noetic-joint-limits-interface \
    ros-noetic-urdf \
    ros-noetic-angles \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-polled-camera \
    ros-noetic-diagnostic-updater \
    ros-noetic-camera-info-manager \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-hector-gazebo-plugins \
    ros-noetic-hector-models \
    && rm -rf /var/lib/apt/lists/*

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

# copy alias bash and add to bashrc
COPY ./alias.sh /
RUN echo "source /alias.sh" >> ~/.bashrc

CMD ["bash"]
