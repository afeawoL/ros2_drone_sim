# ROS2 Drone Simulation Environment

A Dockererized ROS2 Humble development environment for conducting simulatated robotic experimentation involving Drones.

## Project Structure
```
.
├── connect.sh
├── docker
│   └── Dockerfile
├── docker-compose.yml
├── docker_ros_snippets.md
├── LICENSE
├── README.md
├── script_info.txt
├── scripts
│   └── test_gui.sh
├── setup_x11.sh
├── src
│   ├── README.md
│   └── simple_drone_sim
│       ├── LICENSE
│       ├── package.xml
│       ├── resource
│       ├── setup.cfg
│       ├── setup.py
│       ├── simple_drone_sim
│       └── test
└── start.sh
```

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


## Manual Setup (Alternative)

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

***If rviz fails to run, test X11 forwarding:***
  ```bash
  /root/ros2_ws/scripts/test_gui.sh
  ```
## Developing ROS Packages

1. Create or modify ROS2 packages in the `src` directory
2. Build packages inside the container:
   ```bash
   cd /root/ros2_ws
   colcon build
   source install/setup.bash
   ```

### X11 Forwarding (Notes)
  - The setup script (`setup_x11.sh`) configures X11 forwarding for GUI applications
  - If you encounter display issues:
    1. Make sure X11 is running on your host by running: `./setup_x11.sh)`
    2. Restart the container: `docker compose restart`
     
   
