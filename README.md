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

## Documentation

For detailed setup and usage instructions, see the [Setup Instructions](SETUP.md).

## Contributing

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details

