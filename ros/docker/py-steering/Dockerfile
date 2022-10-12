FROM python:3

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update -y && apt upgrade -y libusb-1.0-0
RUN pip install --upgrade pip wheel

COPY ./common/cantools /tmp/cantools
COPY ./common/igvcutils /tmp/igvcutils
RUN pip install /tmp/cantools /tmp/igvcutils && \
    rm -rf /tmp/cantools /tmp/igvcutils

RUN pip install --upgrade odrive
RUN pip install opencan-cand==0.0.5

WORKDIR /app

