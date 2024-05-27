#!/bin/bash
set -e

# Define your desired container name and directory path
ROS_DISTRO="humble"
UBUNTU_DISTRO="jammy"
CONTAINER_NAME="iot-simulation"
MOUNT_DIR="$PWD"
HOST_DIR="$PWD"

if [ "$(docker ps -q -f name=${CONTAINER_NAME})" ]; then
  echo "Container '${CONTAINER_NAME}' is already running."
else
  # Run the Ubuntu container and mount the specified directory
  docker build -f docker/Dockerfile -t $CONTAINER_NAME:0.0.1 . 

  # Enable X11 forwarding for the container
  XSOCK=/tmp/.X11-unix
  XAUTH=/tmp/.docker.xauth
  touch $XAUTH
  xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

  docker run -it \
    --rm \
    --user $(id -u):$(id -g) \
    --group-add sudo \
    --network=host \
    --volume $PWD:$PWD \
    --workdir $PWD \
    --gpus all \
    --name  ${CONTAINER_NAME} \
    --env="DISPLAY=$DISPLAY" \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --env="XAUTHORITY=${XAUTH}" \
    $CONTAINER_NAME:0.0.1
fi
