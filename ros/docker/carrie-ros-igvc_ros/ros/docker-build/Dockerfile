FROM ros:galactic

# install a few things
RUN apt update && apt install -y \
	curl python3-pip

# add the ROS repos
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# upgrade stuff
RUN apt upgrade -y

# install python packages
RUN pip3 install pyserial scipy

# launch a terminal
WORKDIR /carrie
CMD ["bash"]
