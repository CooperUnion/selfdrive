FROM introlab3it/rtabmap_ros:noetic

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /app

RUN apt update && apt install -y git 

RUN apt-get update && apt install -y ros-noetic-robot-state-publisher

RUN mkdir -p ros_ws/src &&\
    git clone https://github.com/stereolabs/zed-ros-interfaces.git ros_ws/src/zed-ros-interfaces 
    
RUN cd ros_ws &&\
    source /opt/ros/noetic/setup.bash &&\
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCATKIN_ENABLE_TESTING=0 -DCMAKE_BUILD_TYPE=Release

# Install pip3
RUN apt-get install -y python3-pip

# copy alias bash and add to bashrc
COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc
RUN echo "source /entrypoint.sh" >> ~/.bashrc

# Install Git and necessary dependencies
RUN apt-get update && apt-get install -y git tesseract-ocr libgl1-mesa-glx ffmpeg libsm6 libxext6 vim

# Set TESSDATA_PREFIX environment variable
ENV TESSDATA_PREFIX '/app/ocr/data/tessdata'

# Define a volume to mount the local directory /home/igvc/selfdrive/ocr/data to /app/ocr/data inside the container
VOLUME ["/home/igvc/selfdrive/rviz/src", "/app/rviz"]

# Change directory to cuber, install the package, and switch back to /app
WORKDIR /app/cuber

COPY ./src /app/cuber

RUN pip install /app/cuber/cuber

WORKDIR /app

# Install the package with setup.py
# RUN pip install ./cuber/cuber

