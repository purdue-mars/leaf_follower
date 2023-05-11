#!/bin/bash

DOCKER_DIR=$(realpath $(dirname $0))
REPO_NAME="leaf-follower"

docker build --build-arg USER=$USER \
    --build-arg ROS_DEPS="$ROS_DEPS" \
    --network=host \
    $M1_ARGS \
    -t $REPO_NAME:local-$USER \
    -f $DOCKER_DIR/Dockerfile \
    $@ \
    $DOCKER_DIR

if [ $? -eq 0 ]; then
    echo "
    Development image built as '$REPO_NAME:local'
then run 'docker commit <container-id>' in a new terminal to make persistent"
else
    echo "
    Error building image, see above messages" >&2
    exit 1
fi

