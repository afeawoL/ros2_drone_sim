#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = "simple_drone_sim"
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Paths to files
    xacro_file = os.path.join(pkg_dir, "description", "drone.urdf.xacro")
    rviz_config = os.path.join(pkg_dir, "rviz", "drone.rviz")
    
    # Launch configuration
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        ),
        
        # Publish robot state
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": xacro_file,
            }],
        ),
        
        # Launch RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
