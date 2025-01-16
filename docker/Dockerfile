FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Set locale
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add the ROS2 apt repository
RUN apt-get update && apt-get install -y curl gnupg lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-ros-base \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && \
    rosdep update

# Create workspace
RUN mkdir -p /root/ros2_ws/src

# Set working directory
WORKDIR /root/ros2_ws

# Source ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Keep container running
CMD ["bash"]
