FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt update && apt install -y git && apt-get -y update

RUN apt install -y python3.9 python3-pip libgl1-mesa-glx --fix-missing

RUN pip install matplotlib opencv-python && pip install numpy --upgrade

RUN apt-get install -y ros-noetic-cv-bridge python3-tk

RUN mkdir -p ros_ws/src

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

# copy alias bash and add to bashrc
COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc
RUN echo "source /entrypoint.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
