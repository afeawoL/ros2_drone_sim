from setuptools import setup
import os
from glob import glob

package_name = "simple_drone_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Drone Developer",
    maintainer_email="developer@example.com",
    description="A simple drone simulation package",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
        ],
    },
)
