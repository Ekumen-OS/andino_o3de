#!/bin/bash

set +e

echo "Running the container..."

CURRENT_FOLDER_PATH_PARENT="$(cd "$(dirname "$0")"; cd .. ; pwd)"
SCRIPT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"

# Read config file: config.yaml and obtain IMAGE_NAME
IMAGE_NAME=$(cat ${SCRIPT_FOLDER_PATH}/config.yaml | grep -v "#" | grep "IMAGE_NAME" | cut -d " " -f 2)
if [ -z "$IMAGE_NAME" ]; then
    echo "Error: IMAGE_NAME not found in the config.yaml file. Using default value."
    IMAGE_NAME="ros_dev_ws_image"
fi
echo "Using Image Name: $IMAGE_NAME"
# Read config file: config.yaml and obtain CONTAINER_NAME
CONTAINER_NAME=$(cat ${SCRIPT_FOLDER_PATH}/config.yaml | grep -v "#" | grep "CONTAINER_NAME" | cut -d " " -f 2)
if [ -z "$CONTAINER_NAME" ]; then
    echo "Error: CONTAINER_NAME not found in the config.yaml file. Using default value."
    CONTAINER_NAME="ros_dev_ws_container"
fi
echo "Using Container Name: $CONTAINER_NAME"

# Read config file: config.yaml and obtain WORKSPACE_FOLDER_NAME
WORKSPACE_FOLDER_NAME=$(cat ${SCRIPT_FOLDER_PATH}/config.yaml | grep -v "#" | grep "WORKSPACE_FOLDER_NAME" | cut -d " " -f 2)
if [ -z "$WORKSPACE_FOLDER_NAME" ]; then
    echo "Error: WORKSPACE_FOLDER_NAME not found in the config.yaml file. Using default value."
    WORKSPACE_FOLDER_NAME="ros_dev_ws"
fi
echo "Using Workspace Folder Name: $WORKSPACE_FOLDER_NAME"

# Location from where the script was executed.
RUN_LOCATION="$(pwd)"

OS_VERSION=focal

SSH_PATH=/home/$USER/.ssh
WORKSPACE_CONTAINER=/home/$(whoami)/$WORKSPACE_FOLDER_NAME/
SSH_AUTH_SOCK_USER=$SSH_AUTH_SOCK

# Check if name container is already taken.
if sudo -g docker docker container ls -a | grep -w "${CONTAINER_NAME}$" -c &> /dev/null; then
   printf "Error: Docker container called $CONTAINER_NAME is already opened.     \
   \n\nTry removing the old container by doing: \n\t docker rm $CONTAINER_NAME   \
   \nor just initialize it with a different name.\n"
   exit 1
fi

echo "Workspace folder is located at $WORKSPACE_CONTAINER"

xhost +
sudo docker run -it \
       --runtime=nvidia \
       --gpus=all \
       -e NVIDIA_VISIBLE_DEVICES=all \
       -e NVIDIA_DRIVER_CAPABILITIES=all \
       -e DISPLAY=$DISPLAY \
       -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK_USER \
       -v $(dirname $SSH_AUTH_SOCK_USER):$(dirname $SSH_AUTH_SOCK_USER) \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v ${CURRENT_FOLDER_PATH_PARENT}:$WORKSPACE_CONTAINER \
       -v $SSH_PATH:$SSH_PATH \
       --privileged \
       --net=host \
       --name $CONTAINER_NAME $IMAGE_NAME
xhost -

# Trap workspace exits and give the user the choice to save changes.
function onexit() {
  while true; do
    read -p "Do you want to overwrite the image called '$IMAGE_NAME' with the current changes? [y/n]: " answer
    if [[ "${answer:0:1}" =~ y|Y ]]; then
      echo "Overwriting docker image..."
      sudo docker commit $CONTAINER_NAME $IMAGE_NAME
      break
    elif [[ "${answer:0:1}" =~ n|N ]]; then
      break
    fi
  done
  docker stop $CONTAINER_NAME > /dev/null
  docker rm $CONTAINER_NAME > /dev/null
}

trap onexit EXIT
