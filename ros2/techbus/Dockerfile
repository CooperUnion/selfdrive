FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

WORKDIR /root/ros_ws

RUN apt-get update && apt-get install -y python3.9 python3-venv python3-pip --fix-missing

RUN apt update && apt upgrade -y

RUN apt install -y \
    cmake \
    curl \
    iputils-ping\
    libglu1-mesa-dev \
    nano \
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

RUN apt autoremove -y \
    && apt clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv .venv
RUN source .venv/bin/activate
RUN python3 -m pip install opencan-cand numpy msgpack cantools

RUN curl -fsSL https://packages.redis.io/gpg | gpg --dearmor -o /usr/share/keyrings/redis-archive-keyring.gpg

RUN echo "deb [signed-by=/usr/share/keyrings/redis-archive-keyring.gpg] https://packages.redis.io/deb $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/redis.list

RUN apt-get update && apt-get install -qy redis-stack-server

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

CMD ["bash"]
