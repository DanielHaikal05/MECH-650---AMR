"""Spawn a single traffic robot and start its bridge + walker.

Args:
  robot_name      — e.g. 'traffic_robot_1' (required, no default to force naming)
  color           — 'red' or 'yellow' (default 'red')
  x, y, z         — spawn position (default x=0, y=0, z=0.30)
  yaw             — spawn yaw (default 0.0)
  world_name      — Gazebo world name (default 'car_world')
  bounds_radius   — soft bounds for the random walker (default 25.0)
  forward_speed   — random walker forward speed (default 1.20, 6x brief spec; tuned during S-6)
  obstacle_threshold — front-sector clearance (default 2.00, tuned during S-6)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _build_actions(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    color = LaunchConfiguration('color').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)
    bounds_radius = LaunchConfiguration('bounds_radius').perform(context)
    forward_speed = LaunchConfiguration('forward_speed').perform(context)
    obstacle_threshold = LaunchConfiguration('obstacle_threshold').perform(context)

    if color not in ('red', 'yellow'):
        raise RuntimeError(f"color must be 'red' or 'yellow', got {color!r}")

    pkg_share = FindPackageShare('amr_traffic').find('amr_traffic')
    sdf_file = (
        f"{pkg_share}/models/traffic_robot_{color}/model.sdf"
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{robot_name}',
        arguments=[
            '-file', sdf_file,
            '-name', robot_name,
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw,
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{robot_name}',
        arguments=[
            # cmd_vel: ROS -> GZ
            f'/model/{robot_name}/cmd_vel'
            f'@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            # odom: GZ -> ROS
            f'/model/{robot_name}/odometry'
            f'@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            # scan: GZ -> ROS
            f'/world/{world_name}/model/{robot_name}/link/base_link/'
            f'sensor/gpu_lidar/scan'
            f'@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        ],
        remappings=[
            (f'/model/{robot_name}/cmd_vel', f'/{robot_name}/cmd_vel'),
            (f'/model/{robot_name}/odometry', f'/{robot_name}/odom'),
            (
                f'/world/{world_name}/model/{robot_name}/link/base_link/'
                f'sensor/gpu_lidar/scan',
                f'/{robot_name}/scan',
            ),
        ],
        output='screen',
    )

    walker = Node(
        package='amr_traffic',
        executable='random_walker_node',
        name='random_walker_node',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'forward_speed': float(forward_speed),
            'turn_speed': 1.00,
            'obstacle_threshold': float(obstacle_threshold),
            'front_sector_half_angle_rad': 0.35,
            'forward_duration_min': 2.0,
            'forward_duration_max': 5.0,
            'turn_duration_min': 1.0,
            'turn_duration_max': 2.5,
            'bounds_radius': float(bounds_radius),
            'loop_rate_hz': 10.0,
        }],
    )

    return [spawn, bridge, walker]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name'),
        DeclareLaunchArgument('color', default_value='red'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.30'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('world_name', default_value='car_world'),
        DeclareLaunchArgument('bounds_radius', default_value='25.0'),
        DeclareLaunchArgument('forward_speed', default_value='0.70'),
        DeclareLaunchArgument('obstacle_threshold', default_value='2.00'),
        OpaqueFunction(function=_build_actions),
    ])
