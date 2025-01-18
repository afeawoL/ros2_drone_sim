import setuptools
from setuptools import setup
import os
from glob import glob

package_name = "simple_drone_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=setuptools.find_packages(include=['simple_drone_sim', 'simple_drone_sim.*']),
    package_data={
        'simple_drone_sim': ['launch/*.py', 'description/*.xacro', 'rviz/*.rviz'],
    },
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
            glob("simple_drone_sim/launch/*.py")),
        ("share/" + package_name + "/description",
            glob("simple_drone_sim/description/*.xacro")),
        ("share/" + package_name + "/rviz",
            glob("simple_drone_sim/rviz/*.rviz")),
    ],
    install_requires=[
        'setuptools',
        'geometry_msgs',
        'nav_msgs',
        'sensor_msgs',
        'tf2_ros',
        'rclpy'
    ],
    zip_safe=False,
    maintainer="Drone Developer",
    maintainer_email="developer@example.com",
    description="A simple drone simulation package",
    license="MIT",
    python_requires='>=3.6',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "drone_controller = simple_drone_sim.drone_controller:main",
            "drone_physics = simple_drone_sim.drone_physics:main",
        ],
    },
)
