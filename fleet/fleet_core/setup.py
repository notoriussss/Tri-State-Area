"""Setup file for fleet_core package."""

from setuptools import find_packages, setup

package_name = "fleet_core"

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
    description="Pure Python core algorithms and hardware abstractions for collector robots.",
    license="MIT",
    tests_require=[
        "pytest",
    ],
    entry_points={
        "console_scripts": [],
    },
)
