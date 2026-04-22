# 1. Start with the official ROS 2 Humble Desktop Full image
FROM osrf/ros:humble-desktop-full

# 2. Update the system and install required robotics packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-ur \
    ros-humble-moveit \
<<<<<<< Updated upstream
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*
=======
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2controlcli \
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/* 
>>>>>>> Stashed changes

# 3. Set up the working directory where your code will live
WORKDIR /ros2_ws

# 4. Make sure ROS 2 is sourced automatically every time you enter the container
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
