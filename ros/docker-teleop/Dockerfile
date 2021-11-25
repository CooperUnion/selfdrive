FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt update && apt install -y git

RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/ros-teleop/teleop_twist_keyboard.git ros_ws/src/teleop_twist

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
