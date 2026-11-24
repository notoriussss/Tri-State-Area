"""Setup file for sim_core package."""

from setuptools import find_packages, setup

package_name = "sim_core"

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
    description="Pure Python 2D arena simulator matching the Tri-State-Area contract.",
    license="MIT",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [],
    },
)
