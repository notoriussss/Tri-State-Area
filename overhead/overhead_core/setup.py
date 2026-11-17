"""Setup file for overhead_core package."""

from setuptools import find_packages, setup

package_name = "overhead_core"

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
    maintainer="Overhead Group",
    maintainer_email="overhead@example.com",
    description="Pure Python core algorithms for the overhead camera vision system.",
    license="MIT",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [],
    },
)
