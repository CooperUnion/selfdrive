FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update && apt install -y git vim

RUN apt-get update && apt install -y python3.9 python3.9-venv python3-pip --fix-missing &&\
    pip install qtpy

RUN mkdir -p ros_ws/src &&\
    git clone -b add-python-bindings https://github.com/ipa-cmh/rviz.git ros_ws/src/rviz

# RUN cd ros_ws &&\
#     source /opt/ros/noetic/setup.bash &&\
#     rosdep install --from-paths src --ignore-src -r -y  &&\
#     catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

# copy alias bash and add to bashrc
COPY ./alias.sh /
COPY ./entrypoint.sh /
RUN echo "source /alias.sh" >> ~/.bashrc
RUN echo "source /entrypoint.sh" >> ~/.bashrc

CMD ["bash"]
