"""Setup configuration for the fleet_ros package."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "fleet_ros"

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
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob(
                os.path.join(
                    "launch",
                    "*launch.[pxy][yma]*",
                )
            ),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob(
                os.path.join(
                    "config",
                    "*.yaml",
                )
            ),
        ),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Fleet Group",
    maintainer_email="fleet@example.com",
    description="ROS 2 nodes and configuration for Phineas and Ferb collector robots.",
    license="MIT",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [
            "collector_node = fleet_ros.collector_node:main",
        ],
    },
)
