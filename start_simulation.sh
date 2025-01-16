#!/bin/bash

# Allow X11 connections
xhost +local:docker

# Build and start the container
docker-compose build
docker-compose up -d

# Enter the container and run the simulation
echo "Starting drone simulation..."
docker-compose exec ros2_drone_sim bash -c "
    source /opt/ros/humble/setup.bash && \
    cd /root/ros2_ws && \
    colcon build && \
    source install/setup.bash && \
    ros2 launch simple_drone_sim display.launch.py
"
