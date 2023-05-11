
CATKIN_WS_MNT="/home/raghava/catkin_ws:/code/catkin_ws"

REPO_NAME="leaf-follower"
CONTAINER_NAME="${CONTAINER_NAME:-$USER-$REPO_NAME}"

docker run --rm -ti --net=host \
    --ipc=host \
    -e DISPLAY=$DISPLAY \
    -v $CATKIN_WS_MNT \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /home/raghava/.Xauthority:/tmp/.XAuthority \
    --device=/dev/video0 \
    --device=/dev/ttyUSB0 \
    -e XAUTHORITY=/tmp/.XAuthority \
    --name $CONTAINER_NAME \
    $REPO_NAME:local-$USER
