#!/bin/bash

# 1. Stop and remove any existing container with the same name
# This prevents the "Name already in use" error
echo "Cleaning up old containers..."
docker rm -f my_ur5e_container 2>/dev/null || true

# 2. Build the image
echo "Building Docker Image..."
docker build -t ur5e_custom_image .

# 3. Grant Display permissions
xhost +local:docker > /dev/null

# 4. Launch the container
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