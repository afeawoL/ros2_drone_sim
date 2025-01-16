#!/bin/bash

# Set up X11 for Docker
export DISPLAY=:1.0
xhost +local:docker

echo "X11 forwarding has been set up for Docker"
echo "Display set to: $DISPLAY"

