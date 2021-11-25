# This is from: https://github.com/introlab/rtabmap_ros/blob/master/docker/noetic/Dockerfile
# FROM ros:noetic-perception

FROM introlab3it/rtabmap_ros:noetic
# FROM ros:noetic-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

#WORKDIR /app

# # install rtabmap packages
# ARG CACHE_DATE=2016-01-01

# RUN apt-get update 

# RUN apt-get install -y \
#     ros-noetic-rtabmap \
#     ros-noetic-rtabmap-ros

COPY ./entrypoint.sh /

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
