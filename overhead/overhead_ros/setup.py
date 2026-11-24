"""Setup configuration for the overhead_ros package."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "overhead_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Overhead Group",
    maintainer_email="overhead@example.com",
    description="ROS 2 adapters for the overhead camera vision and viewer system.",
    license="MIT",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [
            "overhead_node = overhead_ros.overhead_node:main",
            "viewer_node = overhead_ros.viewer_node:main",
            "manipulator_relay_node = overhead_ros.manipulator_relay_node:main",
        ],
    },
)
