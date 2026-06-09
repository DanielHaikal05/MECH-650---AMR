from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_gazebo')
    world_path = os.path.join(pkg_share, 'worlds', 'town.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_path],
            output='screen'
        )
    ])
