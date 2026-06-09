from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # ---------------- Launch arguments ----------------
    exploration_mode_arg = DeclareLaunchArgument(
        'exploration_mode',
        default_value='straight',
        description=(
            'Exploration strategy: straight, free_space, random, '
            'random_small_rotation, landmark_search'
        ),
    )
    exploration_mode = LaunchConfiguration('exploration_mode')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with the mapping configuration',
    )
    use_rviz = LaunchConfiguration('use_rviz')

    # ---------------- Resolve paths ----------------
    gazebo_share      = get_package_share_directory('amr_gazebo')
    mapping_share     = get_package_share_directory('amr_mapping')
    exploration_share = get_package_share_directory('amr_exploration')

    gazebo_launch_file      = os.path.join(gazebo_share,      'launch', 'town.launch.py')
    bridge_launch_file      = os.path.join(gazebo_share,      'launch', 'bridge.launch.py')
    slam_launch_file        = os.path.join(mapping_share,     'launch', 'slam_mapping.launch.py')
    exploration_launch_file = os.path.join(exploration_share, 'launch', 'exploration.launch.py')

    rviz_config_file = os.path.join(mapping_share, 'rviz', 'mapping.rviz')

    # ---------------- Staged startup ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file)
    )

    bridge = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch_file))],
    )

    slam = TimerAction(
        period=10.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_launch_file))],
    )

    exploration = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(exploration_launch_file),
                launch_arguments={
                    'exploration_mode': exploration_mode,
                }.items(),
            )
        ],
    )

    # ---------------- RViz (conditional, immediate) ----------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        exploration_mode_arg,
        use_rviz_arg,
        gazebo,
        bridge,
        slam,
        exploration,
        rviz,
    ])
