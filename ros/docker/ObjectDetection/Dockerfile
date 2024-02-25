FROM pytorch/pytorch:2.1.2-cuda12.1-cudnn8-devel

# Arguments to build Docker Image using CUDA
ARG USE_CUDA=0
ARG TORCH_ARCH=
ARG DEBIAN_FRONTEND=noninteractive

ENV AM_I_DOCKER True
ENV BUILD_WITH_CUDA "${USE_CUDA}"
ENV TORCH_CUDA_ARCH_LIST "${TORCH_ARCH}"
ENV CUDA_HOME /usr/local/cuda-12.1

RUN mkdir -p /home/appuser/GrouningDINO
COPY ./GroundingDINO /home/appuser/GrouningDINO/

RUN apt-get update && apt-get install --no-install-recommends wget ffmpeg \
  libsm6 libxext6 git nano vim libxrender1 libxtst6 -y \
  && apt-get clean && apt-get autoremove && rm -rf /var/lib/apt/lists/*

# Enable "universe" repository
RUN sed -i 's/main$/main universe/' /etc/apt/sources.list

# Install required packages
RUN apt-get update && apt-get install --no-install-recommends -y \
  libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0

ENV PATH /usr/local/cuda-12.1/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/cuda-12.1/lib64:${LD_LIBRARY_PATH}

# Set environment variables for X11
ENV DISPLAY=:0

WORKDIR /home/appuser/GrouningDINO
