from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    exploration_mode = LaunchConfiguration('exploration_mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'exploration_mode',
            default_value='straight',
            description=(
                'Exploration mode: straight, free_space, random, '
                'random_small_rotation, or landmark_search'
            ),
        ),
        Node(
            package='amr_exploration',
            executable='exploration_node',
            name='exploration_node',
            output='screen',
            parameters=[{
                'exploration_mode': exploration_mode,
            }],
        )
    ])
