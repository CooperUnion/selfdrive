FROM python:3

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update -y && apt upgrade -y libusb-1.0-0
RUN pip install --upgrade pip wheel

COPY ./cand /tmp/cand
COPY ./cantools /tmp/cantools
COPY ./igvcutils /tmp/igvcutils
RUN pip install /tmp/cand /tmp/cantools /tmp/igvcutils && \
    rm -rf /tmp/cand /tmp/cantools /tmp/igvcutils

RUN pip install --upgrade odrive

WORKDIR /app
