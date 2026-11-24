"""Launch file for overhead_node, viewer_node, and manipulator_relay_node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates the launch description for the overhead, viewer, and relay nodes."""
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

    use_relay_arg = DeclareLaunchArgument(
        "use_relay",
        default_value="true",
        description="Launch the manipulator relay node between robots and Perry.",
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

    manipulator_relay_node = Node(
        package="overhead_ros",
        executable="manipulator_relay_node",
        name="manipulator_relay_node",
        parameters=[LaunchConfiguration("config_path")],
        condition=IfCondition(LaunchConfiguration("use_relay")),
        output="screen",
    )

    return LaunchDescription(
        [
            use_viewer_arg,
            use_relay_arg,
            config_path_arg,
            overhead_node,
            viewer_node,
            manipulator_relay_node,
        ]
    )
