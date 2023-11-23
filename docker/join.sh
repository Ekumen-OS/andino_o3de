#!/bin/bash

SCRIPT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"

# Read config file: config.yaml and obtain CONTAINER_NAME
CONTAINER_NAME=$(cat ${SCRIPT_FOLDER_PATH}/config.yaml | grep -v "#" | grep "CONTAINER_NAME" | cut -d " " -f 2)
if [ -z "$CONTAINER_NAME" ]; then
    echo "Error: CONTAINER_NAME not found in the config.yaml file. Using default value."
    CONTAINER_NAME="ros_dev_ws_container"
fi
echo "Using Container Name: $CONTAINER_NAME"

sudo docker exec -it $CONTAINER_NAME bash
