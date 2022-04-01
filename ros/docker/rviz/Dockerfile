FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update && apt install -y git 

# install gps driver
RUN apt install -y ros-noetic-rviz

# copy alias bash and add to bashrc
COPY ./alias.sh /
RUN echo "source /alias.sh" >> ~/.bashrc

CMD ["bash"]
