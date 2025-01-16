# ROS2 Drone Simulation Environment

A reproducible ROS2 Humble development environment for drone simulation.

## Repository Structure

```
.
├── docker/
│   └── Dockerfile         # Docker configuration for ROS2 environment
├── src/
│   └── README.md          # Information about ROS2 packages
├── docker-compose.yml     # Docker Compose configuration
├── .gitignore             # Git ignore rules
└── README.md              # Main documentation
```

## Quick Start

1. Clone this repository:
   ```bash
   git clone <repository-url>
   cd ros2_drone_sim
   ```

2. Build and start the environment:
   ```bash
   docker-compose build
   docker-compose up -d
   ```

3. Enter the container:
   ```bash
   docker-compose exec ros2_drone_sim bash
   ```

4. Inside the container, your ROS2 workspace is at `/root/ros2_ws` with the `src` directory mounted from your host.

## Development Workflow

1. Create or modify ROS2 packages in the `src` directory
2. Build packages inside the container:
   ```bash
   cd /root/ros2_ws
   colcon build
   source install/setup.bash
   ```

3. Run your simulation nodes

## Detailed Documentation

For detailed setup and usage instructions, see the [Setup Instructions](SETUP.md).

## Contributing

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details


## Testing GUI Applications

After setting up the environment, you can verify that GUI applications are working correctly:

1. Start the container:
   ```bash
   docker-compose up -d
   ```

2. Run the test script:
   ```bash
   docker-compose exec ros2_drone_sim /root/ros2_ws/scripts/test_gui.sh
   ```

The test script will check:
- Basic X11 forwarding with xeyes
- OpenGL support with glxgears
- ROS2 GUI with RViz2

If any tests fail, ensure:
1. X11 forwarding is properly set up (run ./setup_x11.sh)
2. Your host system supports the required graphics capabilities
3. The container has been rebuilt with latest changes

