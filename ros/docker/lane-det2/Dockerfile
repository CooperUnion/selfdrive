FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update && apt install -y git &&\
    sudo apt-get install ros-noetic-cv-bridge -y &&\
    sudo apt-get install ros-noetic-image-transport -y &&\
    sudo apt-get install software-properties-common -q -y --no-install-recommends &&\
    # sudo add-apt-repository ppa:joseluisblancoc/mrpt-1.5 -y &&\
    # we are assuming that this package used mrpt v1.5.0
    sudo apt-get update 
    # sudo apt-get install libmrpt-dev mrpt-apps -y --no-install-recommends

RUN cd /usr/include &&\
    sudo git clone -b 1.5.0 --depth 1 https://github.com/MRPT/mrpt.git
    # sudo apt upgrade MRPT

# clone ros repo
RUN mkdir -p ros_ws/src

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\ 
    rosdep install --from-paths src --ignore-src -r -y  &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
