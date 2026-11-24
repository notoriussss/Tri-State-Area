"""Launch file for one autonomous collector robot (Phineas or Ferb)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch description for collector_node with per-robot YAML config."""
    fleet_share_dir = get_package_share_directory("fleet_ros")

    robot_arg = DeclareLaunchArgument(
        "robot",
        default_value="phineas",
        description="Robot config name to load: phineas or ferb.",
    )
    driver_arg = DeclareLaunchArgument(
        "motor_driver",
        default_value="fusion_hat",
        description="Motor backend override: fusion_hat, gpio, or fake.",
    )

    config_file = PathJoinSubstitution(
        [
            TextSubstitution(text=os.path.join(fleet_share_dir, "config")),
            [
                LaunchConfiguration("robot"),
                TextSubstitution(text=".yaml"),
            ],
        ]
    )

    collector_node = Node(
        package="fleet_ros",
        executable="collector_node",
        name="collector_node",
        namespace=LaunchConfiguration("robot"),
        parameters=[
            config_file,
            {
                "motor_driver": LaunchConfiguration("motor_driver"),
            },
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            robot_arg,
            driver_arg,
            collector_node,
        ]
    )
