"""Launch file for the AMR Mission Console GUI.

Phase 9 introduced this. Master Plan §3.1 (amr_gui responsibilities).

Launches the mission console GUI. Optionally also launches the mock
mission server for testing without Phase 8 (use the launch arg
'use_mock_server:=true').
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_mock_server = LaunchConfiguration("use_mock_server")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mock_server",
                default_value="false",
                description=(
                    "If true, also launch the mock mission server "
                    "for standalone GUI testing."
                ),
            ),
            Node(
                package="amr_gui",
                executable="mock_mission_server",
                name="mock_mission_server",
                output="screen",
                condition=IfCondition(use_mock_server),
            ),
            Node(
                package="amr_gui",
                executable="mission_console",
                name="mission_console",
                output="screen",
            ),
        ]
    )
