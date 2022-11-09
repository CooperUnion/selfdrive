FROM debian:sid-slim

LABEL maintainer "Cooper IGVC <igvc@cooper.edu>"
LABEL version "0.3.0"

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get upgrade --yes --no-install-recommends \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

ARG BUILD_DIR=/build

# PlatformIO has a very poor dependency pulling system so we need to
# "trick" the cli tool into pulling everything we need to build node
# firmware.  To do this we copy the node source tree to get a copy of
# our current configuration.
ADD . $BUILD_DIR
WORKDIR $BUILD_DIR

RUN pip install --upgrade pip \
    && pip install platformio \
    &&  pio run --verbose --target compiledb \
    && yes | platformio system prune \
    && rm -rf *
