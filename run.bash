#!/bin/bash

docker run --rm -it --gpus all \
    --privileged \
    --network=host \
    -u $UID \
    -e RUN=true \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --name dtc-jackal-$(hostname)-glider \
    dtc-jackal-$(hostname):glider