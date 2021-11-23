FROM ros:noetic-ros-core-focal

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt update &&\
    apt install -y ros-noetic-rqt ros-noetic-rqt-common-plugins

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
