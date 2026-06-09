from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    amr_mapping_share = get_package_share_directory('amr_mapping')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    slam_params_file = os.path.join(
        amr_mapping_share,
        'config',
        'slam_params.yaml'
    )

    slam_launch_file = os.path.join(
        slam_toolbox_share,
        'launch',
        'online_async_launch.py'
    )

    odom_to_tf_node = Node(
        package='amr_mapping',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    static_lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_lidar_tf_publisher',
        output='screen',
        arguments=[
            '0.85', '0', '0.55',
            '0', '0', '0',
            'vehicle_blue/chassis',
            'vehicle_blue/chassis/gpu_lidar'
        ],
        parameters=[
            {'use_sim_time': True}
        ]
    )

    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params_file,
        }.items()
    )

    return LaunchDescription([
        odom_to_tf_node,
        static_lidar_tf_node,
        slam_toolbox_node,
    ])
