FROM ros:noetic-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /app

RUN apt update && apt install -y curl

RUN apt-get -y update

# RUN apt install -y python3-pip --fix-missing

# RUN pip install transformations

RUN apt-get install -y python3-tf2-ros

RUN apt-get install -y python3-tf-conversions

RUN curl -fsSL https://packages.redis.io/gpg | gpg --dearmor -o /usr/share/keyrings/redis-archive-keyring.gpg

RUN echo "deb [signed-by=/usr/share/keyrings/redis-archive-keyring.gpg] https://packages.redis.io/deb $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/redis.list

RUN apt-get update && apt-get install -qy redis

RUN mkdir -p ros_ws/src

RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release
    # chmod +x /app/ros_ws/src/encoder_odom_launch/src/encoder_odom.py

COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]