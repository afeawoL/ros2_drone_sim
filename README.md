# ROS2 Drone Simulation Environment

A Docker w/ a ROS2 Humble development environment for drone simulation with X11 forwarding.

## Repository Structure

```
.
├── docker/
│   └── Dockerfile         # Docker configuration for ROS2 environment
├── src/
│   └── README.md          # Information about ROS2 packages
├── docker-compose.yml     # Docker Compose configuration
├── setup_x11.sh           # X11 forwarding setup script
├── README.md              # Main documentation             
├── LICENSE                # MIT License
└── .gitignore

```

## Quick Start

1. Clone this repository:
   ```bash
   git clone https://github.com/afeawoL/ros2_drone_sim/
   cd ros2_drone_sim
   ```

2. Set up X11 forwarding (required for GUI applications):
   ```bash
   ./setup_x11.sh
   ```

3. Build and start the environment:
   ```bash
   docker compose build
   docker compose up -d
   ```

4. Enter the docker container:
   ```bash
   docker compose exec ros2_drone_sim bash
   ```

5. Run rviz to test X11 fowarding:
   ```bash
   rviz2
   ```

   If rviz fails to run, run test_gui.sh to verify that X11 forwarding is active:
   ```bash
   docker compose up -d && docker compose exec ros2_drone_sim /root/ros2_ws/scripts/test_gui.sh
   ```

## X11 Forwarding Notes

- The setup script (`setup_x11.sh`) configures X11 forwarding for GUI applications
- If you encounter display issues:
  1. Make sure X11 is running on your host by running: `./setup_x11.sh)`
  2. Restart the container: `docker compose restart`
     
## Developing ROS Packages

1. Create or modify ROS2 packages in the `src` directory
2. Build packages inside the container:
   ```bash
   cd /root/ros2_ws
   colcon build
   source install/setup.bash
   ```


