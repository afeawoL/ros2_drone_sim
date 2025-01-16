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
├── .gitignore             # Git ignore rules
├── LICENSE                # MIT License
├── SETUP.md               # Detailed setup instructions
└── README.md              # Main documentation
```

## Quick Start

1. Clone this repository:
   ```bash
   git clone <repository-url>
   cd ros2_drone_sim
   ```

2. Set up X11 forwarding (required for GUI applications):
   ```bash
   ./setup_x11.sh
   ```

3. Build and start the environment:
   ```bash
   docker-compose build
   docker-compose up -d
   ```

4. Enter the docker container:
   ```bash
   docker-compose exec ros2_drone_sim bash
   ```

5. Test GUI applications (inside docker container):
   ```bash
   # Run RViz2 to test GUI
   rviz2
   ```

## Development Workflow

1. Create or modify ROS2 packages in the `src` directory
2. Build packages inside the container:
   ```bash
   cd /root/ros2_ws
   colcon build
   source install/setup.bash
   ```

3. Run your simulation nodes and visualization tools

## X11 Forwarding Notes

- The setup script (`setup_x11.sh`) configures X11 forwarding for GUI applications
- If you encounter display issues:
  1. Make sure X11 is running on your host
  2. Run `./setup_x11.sh` again
  3. Restart the container: `docker-compose restart`

## Documentation

For detailed setup instructions read [Setup Instructions](SETUP.md).
