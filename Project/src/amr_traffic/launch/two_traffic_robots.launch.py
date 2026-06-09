"""Spawn two traffic robots in the existing town world.

Args:
  world_name              — default 'car_world'
  robot1_x, robot1_y      — defaults (-5.0, 5.0)
  robot1_yaw              — default 1.5708
  robot2_x, robot2_y      — defaults (5.0, -5.0)
  robot2_yaw              — default 0.0
  bounds_radius           — random-walker soft bounds around each spawn
  forward_speed           — random-walker forward speed
  obstacle_threshold      — random-walker obstacle clearance
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('amr_traffic')
    spawn_one = PathJoinSubstitution([pkg, 'launch', 'spawn_one_traffic_robot.launch.py'])

    args = [
        DeclareLaunchArgument('world_name', default_value='car_world'),
        DeclareLaunchArgument('robot1_x', default_value='0.0'),
        DeclareLaunchArgument('robot1_y', default_value='5.0'),
        DeclareLaunchArgument('robot1_yaw', default_value='1.5708'),
        DeclareLaunchArgument('robot2_x', default_value='5.0'),
        DeclareLaunchArgument('robot2_y', default_value='0.0'),
        DeclareLaunchArgument('robot2_yaw', default_value='0.0'),
        DeclareLaunchArgument('bounds_radius', default_value='25.0'),
        DeclareLaunchArgument('forward_speed', default_value='0.70'),
        DeclareLaunchArgument('obstacle_threshold', default_value='2.00'),
    ]

    spawn_robot_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_one),
        launch_arguments={
            'robot_name': 'traffic_robot_1',
            'color': 'red',
            'x': LaunchConfiguration('robot1_x'),
            'y': LaunchConfiguration('robot1_y'),
            'yaw': LaunchConfiguration('robot1_yaw'),
            'world_name': LaunchConfiguration('world_name'),
            'bounds_radius': LaunchConfiguration('bounds_radius'),
            'forward_speed': LaunchConfiguration('forward_speed'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
        }.items(),
    )

    spawn_robot_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_one),
        launch_arguments={
            'robot_name': 'traffic_robot_2',
            'color': 'yellow',
            'x': LaunchConfiguration('robot2_x'),
            'y': LaunchConfiguration('robot2_y'),
            'yaw': LaunchConfiguration('robot2_yaw'),
            'world_name': LaunchConfiguration('world_name'),
            'bounds_radius': LaunchConfiguration('bounds_radius'),
            'forward_speed': LaunchConfiguration('forward_speed'),
            'obstacle_threshold': LaunchConfiguration('obstacle_threshold'),
        }.items(),
    )

    # Stagger by 2 s so the two `ros_gz_sim create` calls don't race.
    return LaunchDescription(args + [
        spawn_robot_1,
        TimerAction(period=2.0, actions=[spawn_robot_2]),
    ])
