"""Setup configuration for the sim_ros package."""

from setuptools import find_packages, setup

package_name = "sim_ros"

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
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Fleet Group",
    maintainer_email="fleet@example.com",
    description="ROS 2 node publishing the simulated overhead and Perry contract.",
    license="MIT",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [
            "sim_node = sim_ros.sim_node:main",
        ],
    },
)
