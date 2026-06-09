from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    amr_navigation_share = get_package_share_directory('amr_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(
        amr_navigation_share,
        'maps',
        'my_town_map1.yaml'
    )

    params_file = os.path.join(
        amr_navigation_share,
        'config',
        'nav2_params.yaml'
    )

    nav2_launch_file = os.path.join(
        nav2_bringup_share,
        'launch',
        'bringup_launch.py'
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_file,
            'params_file': params_file,
            'autostart': 'true',
        }.items()
    )

    static_lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_tf_publisher',
        output='screen',
        arguments=[
            '0', '0', '1.0',
            '0', '0', '0',
            'vehicle_blue/chassis',
            'vehicle_blue/chassis/gpu_lidar'
        ],
        parameters=[
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        static_lidar_tf_node,
        nav2_bringup
    ])
