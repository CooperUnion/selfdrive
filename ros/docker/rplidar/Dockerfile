FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt update && apt install -y git ros-noetic-rviz

RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/Slamtec/rplidar_ros.git ros_ws/src/rplidar_ros

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
