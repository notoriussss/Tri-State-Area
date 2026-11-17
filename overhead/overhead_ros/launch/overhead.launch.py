"""Launch file for overhead_node and optional viewer_node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch description for the overhead camera and viewer nodes."""
    overhead_share_dir = get_package_share_directory("overhead_ros")
    default_config_path = os.path.join(
        overhead_share_dir,
        "config",
        "overhead.yaml",
    )

    use_viewer_arg = DeclareLaunchArgument(
        "use_viewer",
        default_value="true",
        description="Launch the on-screen OpenCV viewer visualization node.",
    )

    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="Path to overhead configuration YAML file.",
    )

    overhead_node = Node(
        package="overhead_ros",
        executable="overhead_node",
        name="overhead_node",
        parameters=[LaunchConfiguration("config_path")],
        output="screen",
    )

    viewer_node = Node(
        package="overhead_ros",
        executable="viewer_node",
        name="viewer_node",
        condition=IfCondition(LaunchConfiguration("use_viewer")),
        output="screen",
    )

    return LaunchDescription(
        [
            use_viewer_arg,
            config_path_arg,
            overhead_node,
            viewer_node,
        ]
    )
