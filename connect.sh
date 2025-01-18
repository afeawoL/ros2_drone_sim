#!/bin/bash
set -e

# Color definitions
GREEN="\e[0;32m"
RED="\e[0;31m"
NC="\e[0m"

# Find all potential drone simulation containers
CONTAINERS=$(docker ps --filter "name=ros2_drone" --format "{{.ID}} {{.Names}}")

if [ -z "$CONTAINERS" ]; then
    echo -e "${RED}Error: No running drone simulation containers found${NC}"
    echo "Please run ./start.sh first to start the simulation environment"
    exit 1
fi

# If multiple containers found, let user choose
container_count=$(echo "$CONTAINERS" | wc -l)
if [ "$container_count" -gt 1 ]; then
    echo "Multiple containers found:"
    echo "$CONTAINERS" | nl
    read -p "Select container number (1-$container_count): " selection
    CONTAINER_ID=$(echo "$CONTAINERS" | sed -n "${selection}p" | cut -d' ' -f1)
else
    CONTAINER_ID=$(echo "$CONTAINERS" | cut -d' ' -f1)
fi

# Connect to the container with ROS2 environment
echo -e "${GREEN}Connecting to drone simulation environment...${NC}"
docker exec -it $CONTAINER_ID bash -c "source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && exec bash"
