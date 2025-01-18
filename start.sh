#!/bin/bash
set -e

echo "🚀 Starting ROS2 Drone Simulation Environment..."

# Enable X11 forwarding
if ! xhost +local:docker > /dev/null 2>&1; then
    echo "❌ Failed to enable X11 forwarding. Please ensure X11 is running."
    exit 1
fi

# Ensure container is running
echo "🔄 Setting up Docker container..."
docker compose up -d

# Build and launch simulation
echo "🛠️  Building and launching simulation..."
docker compose exec ros2_drone_sim bash -c "
    source /opt/ros/humble/setup.bash && \
    cd /root/ros2_ws && \
    colcon build --packages-select simple_drone_sim && \
    source install/setup.bash && \
    ros2 launch simple_drone_sim display.launch.py
"
