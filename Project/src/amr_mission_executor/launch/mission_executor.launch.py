"""Launch file for the mission executor.

Starts the mission_executor node with landmarks loaded from
config/landmarks.yaml. Use this alongside (not instead of) Nav2 and the
GUI — see the top-level bringup launch in Step 6 for the full system.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("amr_mission_executor")
    default_landmarks = os.path.join(pkg_share, "config", "landmarks.yaml")

    landmarks_arg = DeclareLaunchArgument(
        "landmarks_file",
        default_value=default_landmarks,
        description="Path to the landmarks YAML parameter file.",
    )

    mission_executor_node = Node(
        package="amr_mission_executor",
        executable="mission_executor",
        name="mission_executor",
        output="screen",
        parameters=[LaunchConfiguration("landmarks_file")],
    )

    return LaunchDescription([
        landmarks_arg,
        mission_executor_node,
    ])
