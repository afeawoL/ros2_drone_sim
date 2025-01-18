# ROS2 Drone Simulation Environment

A containerized ROS2 Humble development environment for drone simulation with X11 forwarding.

## Quick Start (Recommended)

1. Clone this repository:
```bash
git clone https://github.com/afeawoL/ros2_drone_sim/
cd ros2_drone_sim
```

2. Make the start script executable:
```bash
chmod +x start.sh
```

3. Run the simulation:
```bash
./start.sh
```

The start script will automatically:
- Enable X11 forwarding
- Start the Docker container
- Build the ROS2 packages
- Launch the drone simulation with visualization

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

## Manual Setup (Alternative)

If you need more control over the setup process:

1. Start the Docker container:
```bash
docker compose build
docker compose up -d
```

2. Enter the container:
```bash
docker compose exec ros2_drone_sim bash
```

3. Inside the container, build and run:
```bash
cd /root/ros2_ws
colcon build --packages-select simple_drone_sim
source install/setup.bash
ros2 launch simple_drone_sim display.launch.py
```

If rviz fails to run, test X11 forwarding:
```bash
/root/ros2_ws/scripts/test_gui.sh
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


