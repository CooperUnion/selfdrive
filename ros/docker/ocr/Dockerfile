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

# Copy the OCR requirements.txt file into the container at /app/ocr/requirements.txt
COPY ./src/requirements.txt /app/ocr/requirements.txt

# Install OCR dependencies
RUN pip install -r /app/ocr/requirements.txt

# copy alias bash and add to bashrc
COPY ./entrypoint.sh /
COPY ./alias.sh /

ENV TESSDATA_PREFIX '/app/ocr/data/tessdata'

RUN echo "source /alias.sh" >> ~/.bashrc
RUN echo "source /entrypoint.sh" >> ~/.bashrc

# Install Git and necessary dependencies
RUN apt-get update && apt-get install -y git tesseract-ocr libgl1-mesa-glx ffmpeg libsm6 libxext6 vim

# Clone the pytesseract repository
RUN git clone https://github.com/madmaze/pytesseract.git

# Change directory to pytesseract and install it
WORKDIR /app/pytesseract
RUN pip install -U .

# Change back to the /app directory
WORKDIR /app

# Install any necessary dependencies
# If you have a requirements.txt file, uncomment the following line
# COPY requirements.txt /app/requirements.txt
# RUN pip install --no-cache-dir -r requirements.txt

# Set TESSDATA_PREFIX environment variable
ENV TESSDATA_PREFIX '/app/ocr/data/tessdata'

# Define a volume to mount the local directory /home/igvc/data to /app/data inside the container
VOLUME ["/home/igvc/selfdrive/ros/docker/src/data", "/app/ocr/data"]
VOLUME ["/home/igvc/selfdrive/ros/docker/src", "/app/ocr"]

# Define a volume to mount the local directory /home/igvc/selfdrive/ocr/data to /app/ocr/data inside the container
VOLUME ["/home/igvc/selfdrive/ocr/data", "/app/ocr/data"]
VOLUME ["/home/igvc/selfdrive/ocr", "/app/ocr"]
