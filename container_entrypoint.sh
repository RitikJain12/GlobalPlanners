#!/bin/bash

apt-get update && apt-get install -y \
    xorg \
    openbox

xhost +local:docker

docker run --rm -v /tmp/.X11-unix:/mnt alpine true