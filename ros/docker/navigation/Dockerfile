FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# install dep and such
RUN apt update && apt upgrade -y
RUN apt install -y git

# clone ros repos
RUN mkdir -p ros_ws/src &&\
    git clone  --branch 1.17.2 https://github.com/ros-planning/navigation.git ros_ws/src/navigation &&\
    git clone https://github.com/rst-tu-dortmund/teb_local_planner.git ros_ws/src/teb_local_planner &&\
        cd ros_ws/src/teb_local_planner && git checkout 0e839074c3407ff9ee2206a49567a20bd49fd8cc

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\ 
    rosdep install --from-paths src --ignore-src -r -y  &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
