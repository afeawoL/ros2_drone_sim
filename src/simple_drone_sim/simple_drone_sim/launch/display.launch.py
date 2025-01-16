#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = "simple_drone_sim"
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Paths to files
    xacro_file = os.path.join(pkg_dir, "description", "drone.urdf.xacro")
    rviz_config = os.path.join(pkg_dir, "rviz", "drone.rviz")
    
    # Robot description in URDF from XACRO
    robot_description = Command([
        "xacro ",
        xacro_file
    ])

    return LaunchDescription([
        # Publish robot state
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
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
