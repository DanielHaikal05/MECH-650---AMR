"""Standalone traffic-robot demo: town world + traffic robots.

NOT for use in the integrated mission stack — that's amr_bringup's job (Phase 11).
This is purely for testing Phase 10 in isolation.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    town_launch = PathJoinSubstitution([
        FindPackageShare('amr_gazebo'), 'launch', 'town.launch.py'
    ])
    traffic_launch = PathJoinSubstitution([
        FindPackageShare('amr_traffic'), 'launch', 'two_traffic_robots.launch.py'
    ])

    town = IncludeLaunchDescription(PythonLaunchDescriptionSource(town_launch))
    traffic = IncludeLaunchDescription(PythonLaunchDescriptionSource(traffic_launch))

    # Wait long enough that the world has loaded before spawning extra robots.
    return LaunchDescription([
        town,
        TimerAction(period=12.0, actions=[traffic]),
    ])
