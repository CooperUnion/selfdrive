#FROM ros:noetic-ros-base
FROM ros:indigo

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /app 

RUN cd /usr/include &&\
    sudo add-apt-repository ppa:joseluisblancoc/mrpt-1.5
    sudo apt-get update 
    sudo apt-get install libmrpt-dev mrpt-apps -y

# RUN cd /usr/include &&\
#    sudo git clone -b 1.5.0 --depth 1 https://github.com/MRPT/mrpt.git.


# install dep and such
RUN apt update && apt install -y git &&\
    apt-get install ros-indigo-cv-bridge -y &&\
    apt-get install ros-indigo-image-transport -y &&\
    apt-get install software-properties-common -q -y --no-install-recommends &&\
    

    # sudo add-apt-repository ppa:joseluisblancoc/mrpt-1.5 -y &&\
    # we are assuming that this package used mrpt v1.5.0
    # sudo apt-get install libmrpt-dev mrpt-apps -y --no-install-recommends


# clone ros repo
RUN mkdir -p ros_ws/src

RUN cd ros_ws &&\
    source /opt/ros/indigo/setup.bash &&\ 
    rosdep install --from-paths src --ignore-src -r -y  &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
