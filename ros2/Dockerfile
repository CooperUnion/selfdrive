FROM osrf/ros:humble-desktop-full

WORKDIR /root/humble_dev_ws

RUN sudo apt-get update && apt-get install -y \
    cmake \
    curl \
    ros-humble-imu-tools \
    iputils-ping\
    libglu1-mesa-dev \
    nano \
    python3-pip \
    python3-pydantic \
    ros-dev-tools \
    ruby-dev \
    rviz \
    tmux \
    wget \
    xorg-dev \ 
    vim 

RUN pip3 install setuptools==58.2.0
RUN pip3 install python-statemachine pyserial

RUN wget https://github.com/openrr/urdf-viz/releases/download/v0.38.2/urdf-viz-x86_64-unknown-linux-gnu.tar.gz && \
    tar -xvzf urdf-viz-x86_64-unknown-linux-gnu.tar.gz -C /usr/local/bin/ && \
    chmod +x /usr/local/bin/urdf-viz && \
    rm -f urdf-viz-x86_64-unknown-linux-gnu.tar.gz

RUN apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN echo "export DISABLE_AUTO_TITLE=true" >> /root/.bashrc
RUN echo 'LC_NUMERIC="en_US.UTF-8"' >> /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN echo 'alias rosdi="rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y"' >> /root/.bashrc
RUN echo 'alias cbuild="colcon build --symlink-install"' >> /root/.bashrc
RUN echo 'alias ssetup="source ./install/local_setup.bash"' >> /root/.bashrc

# RUN echo "autoload -U bashcompinit" >> /root/.bashrc 
# RUN echo "bashcompinit" >> /root/.bashrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> /root/.bashrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> /root/.bashrc

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc 
RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> /root/.bashrc
RUN echo 'export ROS_DOMAIN_ID=0' >> /root/.bashrc

RUN apt update && apt upgrade -y

CMD ["bash"]
