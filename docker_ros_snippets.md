
## INSTALL ROS2:HUMBLE ##

# Enable ubuntu universe repo
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key with apt:
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ros2 repository to sources:
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update apt repository & install ros2:
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# Source ros2 for initial run:
source /opt/ros/humble/setup.bash

------------------------------------------------------------------------------------------------------------

## INSTALL GAZEBO ##
sudo apt-get install ros-${ROS_DISTRO}-ros-gz

## SET-UP X11 ON HOST/MAIN UBUNTU ##
export DISPLAY=:1.0
xhost +local:docker

## ROS2:HUMBLE DOCKER RUN COMMAND ##
docker run -it --net=host -e DISPLAY=$DISPLAY \
    -v $HOME/.Xauthority:/root/.Xauthority:ro osrf/ros:humble-desktop

## CHECK ROS VERSION ##
echo $ROS_DISTRO


## LISTENER/TALKER TESTING ##
#  Run talker in docker:
ros2 run demo_nodes_cpp talker
o
# Run listener on host/main ubuntu:
ros2 run demo_nodes_cpp listener

# Run rqt to visualize listener/talker:
rqt # Then Navigate through Plugins > Introspection > Node Graph

# Output active ros2 topics:
ros2 topic list

ros2 run demo_nodes_cpp listener

### INSTALL DOCKER USING THE APT REPOSITORY ###

## Set up Docker's apt repository ##
	
# Add Docker's official GPG key:
	sudo apt-get update
	sudo apt-get install ca-certificates curl
	sudo install -m 0755 -d /etc/apt/keyrings
	sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
	sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
	echo \	
		"deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
		$(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
		sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
	sudo apt-get update
	
	
# Install the latest Docker packages
	sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

-------------------------------------------------------------------------------------------------------------

docker exec -it <docker-container-name> bash

docker exec -it <docker-container-name> sh



