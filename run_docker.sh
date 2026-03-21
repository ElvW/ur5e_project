#!/bin/bash

# 1. Build the image from your Dockerfile
# It tags (-t) the image as 'ur5e_custom_image'
echo "Building Docker Image..."
docker build -t ur5e_custom_image .

# 2. Grant Docker permission to use your Windows/WSL Display
xhost +local:docker

# 3. Launch the container
echo "Starting the Robot Environment..."
docker run -it \
    --name my_ur5e_container \
    --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/ros2_ws" \
    ur5e_custom_image bash
