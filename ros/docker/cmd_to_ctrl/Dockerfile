FROM ros:noetic-ros-base

# prevents prompts from popping up during installation
# for certain packages
ARG DEBIAN_FRONTEND=noninteractive

# install basic utils
RUN apt update && \
    apt install -y curl vim

# install python3.10 
RUN apt update && \
    apt install -y software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt install -y python3.10 python3.10-distutils

# priortize python3.10 for python/python3 commands
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3.10 1 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1

# install pip for python3.10
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python

WORKDIR /app

RUN mkdir ros_ws

COPY ./entrypoint.sh /
COPY ./alias.sh /

RUN echo "source /alias.sh" >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
