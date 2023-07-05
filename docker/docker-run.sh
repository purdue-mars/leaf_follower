#!/bin/bash
function abspath() {
    # generate absolute path from relative path
    # $1     : relative filename
    # return : absolute path
    if [ -d "$1" ]; then
        # dir
        (cd "$1"; pwd)
    elif [ -f "$1" ]; then
        # file
        if [[ $1 = /* ]]; then
            echo "$1"
        elif [[ $1 == */* ]]; then
            echo "$(cd "${1%/*}"; pwd)/${1##*/}"
        else
            echo "$(pwd)/$1"
        fi
    fi
}

REPO_DIR=$(abspath "..")
REPO_MNT="$REPO_DIR:/code/catkin_ws/src/leaf_follower"

# For GUI

X11_MNT="/tmp/.X11-unix:/tmp/.X11-unix"
XAuthority_MNT="/home/phenoabe/.Xauthority:/tmp/.XAuthority" # change this

# device passthrough to container
GELSIGHT_DEV="/dev/video0" # use v4l2-ctl
GRIPPER_USB_DEV="/dev/ttyUSB0"

REPO_NAME="raghavauppuluri/leaf-follower"
CONTAINER_NAME="leaf-follower-$USER"

docker run --rm -ti --net=host \
    --ipc=host \
    -e DISPLAY=$DISPLAY \
    -v $REPO_MNT \
    -v $X11_MNT \
    -v $XAuthority_MNT \
    --device=$GELSIGHT_DEV \
    --device=$GRIPPER_USB_DEV \
    -e XAUTHORITY=/tmp/.XAuthority \
    --name $CONTAINER_NAME \
    $REPO_NAME:latest
