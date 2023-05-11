REPO_NAME="leaf-follower"
CONTAINER_NAME="${CONTAINER_NAME:-$USER-$REPO_NAME}"

docker exec -it \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=/tmp/.XAuthority \
    -e LIBGL_ALWAYS_INDIRECT \
    $@ \
    $CONTAINER_NAME \
    ${DOCKER_CMD:-/bin/bash}
