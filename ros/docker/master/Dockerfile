FROM ros:noetic-ros-core-focal

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt update &&\
    apt install -y ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-ros-tutorials ros-noetic-geometry-tutorials ros-noetic-rviz ros-noetic-rosbash ros-noetic-rqt-tf-tree

# copy alias bash and add to bashrc
COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc
RUN echo "source /entrypoint.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
