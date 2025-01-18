#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
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

    # Physics parameters
    # Physics simulation parameters
    gravity_arg = DeclareLaunchArgument(
        'gravity',
        default_value='-9.81',
        description='Gravity acceleration (m/s^2)'
    )

    air_density_arg = DeclareLaunchArgument(
        'air_density',
        default_value='1.225',
        description='Air density (kg/m^3)'
    )

    drag_coefficient_arg = DeclareLaunchArgument(
        'drag_coefficient',
        default_value='0.47',
        description='Drag coefficient'
    )

    # PID Control Parameters
    pid_p_arg = DeclareLaunchArgument(
        'pid_p',
        default_value='[2.0, 2.0, 2.0]',
        description='PID Proportional gains [x, y, z]'
    )

    pid_i_arg = DeclareLaunchArgument(
        'pid_i',
        default_value='[0.1, 0.1, 0.1]',
        description='PID Integral gains [x, y, z]'
    )

    pid_d_arg = DeclareLaunchArgument(
        'pid_d',
        default_value='[0.5, 0.5, 0.5]',
        description='PID Derivative gains [x, y, z]'
    )

    # Simulation rate parameters
    sim_rate_arg = DeclareLaunchArgument(
        'sim_rate',
        default_value='100.0',
        description='Physics simulation rate in Hz'
    )

    # Launch argument for enabling/disabling the controller
    use_controller_arg = DeclareLaunchArgument(
        'use_controller',
        default_value='true',
        description='Enable/disable the drone controller'
    )

    return LaunchDescription([
        # Launch Arguments
        gravity_arg,
        air_density_arg,
        drag_coefficient_arg,
        use_controller_arg,
        pid_p_arg,
        pid_i_arg,
        pid_d_arg,
        sim_rate_arg,
        
        # Create namespace for the drone
        PushRosNamespace('drone'),
        
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

        # Joint State Publisher GUI for manual joint control simulation
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),

        # Drone Controller Node (conditional launch)
        # Drone Controller
        Node(
            package='simple_drone_sim',
            executable='drone_controller',
            name='controller',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_controller')),
            parameters=[{
                'max_velocity': 2.0,
                'max_altitude': 5.0,
                'control_rate': 50.0,
                'pid': {
                    'p': LaunchConfiguration('pid_p'),
                    'i': LaunchConfiguration('pid_i'),
                    'd': LaunchConfiguration('pid_d')
                }
            }],
            remappings=[
                ('/cmd_vel', '/drone/cmd_vel'),
                ('/odom', '/drone/odom'),
                ('/imu', '/drone/imu')
            ]
        ),

        # Physics Simulator Node
        Node(
            package='simple_drone_sim',
            executable='drone_physics',
            name='physics',
            output='screen',
            parameters=[{
                'gravity': LaunchConfiguration('gravity'),
                'air_density': LaunchConfiguration('air_density'),
                'drag_coefficient': LaunchConfiguration('drag_coefficient'),
                'simulation_rate': LaunchConfiguration('sim_rate'),
                'robot_mass': 1.5,
                'rotor_coefficient': 1e-5,
                'moment_of_inertia': [0.0823, 0.0823, 0.149],
                'max_thrust': 20.0,
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            remappings=[
                ('cmd_vel', 'physics/cmd_vel'),
                ('odom', 'odom'),
                ('imu', 'imu')
            ]
        ),

        # Static Transform Publisher for drone base frame
        # Static transforms group
        GroupAction(
            actions=[
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='world_to_base_link',
                    namespace='tf',
                    parameters=[{
                        'translation': [0.0, 0.0, 0.0],
                        'rotation': [0.0, 0.0, 0.0, 1.0],
                        'frame_id': 'world',
                        'child_frame_id': 'drone_base_link'
                    }]
                ),
            ]
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
