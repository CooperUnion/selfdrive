FROM debian:bullseye-slim

LABEL maintainer "Cooper IGVC <igvc@cooper.edu>"
LABEL version "0.4.0"

SHELL ["/bin/bash", "-c"]

ARG BUILD_DIR=/build

RUN apt-get update \
    && apt-get upgrade --yes \
    python3.9 python3.9-venv python3.9-distutils \
    curl make git \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py \
    && python3.9 get-pip.py \
    && python3.9 -m pip install --upgrade pip

ADD . $BUILD_DIR
WORKDIR $BUILD_DIR
RUN git clean -ffdx

# cache some pip downloads by doing this, and get platformio
RUN make dependencies

WORKDIR dbw/node_fw

RUN python3.9 -m platformio pkg install \
    && yes | python3.9 -m platformio system prune

WORKDIR $BUILD_DIR
RUN rm -rf *
