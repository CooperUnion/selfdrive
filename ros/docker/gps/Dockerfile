FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update && apt install -y git

# install gps driver
RUN apt install -y ros-noetic-novatel-oem7-driver

# copy alias bash and add to bashrc
COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc
RUN echo "source /entrypoint.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
