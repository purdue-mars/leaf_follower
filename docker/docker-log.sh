CONTAINER_NAME="leaf-follower-$USER"

docker exec -it \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=/tmp/.XAuthority \
    -e LIBGL_ALWAYS_INDIRECT \
    $@ \
    $CONTAINER_NAME \
    ${DOCKER_CMD:-/bin/bash -i}
