"""Top-level bringup for the full AMR service-robot system.

Composes Gazebo + bridge + Nav2 + mission executor + GUI, plus
optional traffic robots and RViz.

Args:
  use_rviz:=true        Also launch RViz2.
  auto_init_pose:=false Skip auto-publish of /initialpose.
  use_traffic:=true     Spawn two traffic robots as dynamic obstacles.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # ---- Resolve sub-launch paths ----
    gazebo_share = get_package_share_directory("amr_gazebo")
    nav_share = get_package_share_directory("amr_navigation")
    executor_share = get_package_share_directory("amr_mission_executor")
    gui_share = get_package_share_directory("amr_gui")
    traffic_share = get_package_share_directory("amr_traffic")

    # ---- Launch arguments ----
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="If true, also launch RViz2.",
    )
    auto_init_pose_arg = DeclareLaunchArgument(
        "auto_init_pose",
        default_value="true",
        description=(
            "If true, auto-publish an initial pose to /initialpose 8s "
            "after launch so AMCL doesn't need a manual click."
        ),
    )
    use_traffic_arg = DeclareLaunchArgument(
        "use_traffic",
        default_value="false",
        description=(
            "If true, spawn two traffic robots (red + yellow) as "
            "dynamic obstacles. Spawning is delayed so Gazebo has time "
            "to load fully."
        ),
    )
    traffic_robot1_x_arg = DeclareLaunchArgument(
        "traffic_robot1_x",
        default_value="-8.0",
        description="Traffic robot 1 spawn x position.",
    )
    traffic_robot1_y_arg = DeclareLaunchArgument(
        "traffic_robot1_y",
        default_value="-3.0",
        description="Traffic robot 1 spawn y position.",
    )
    traffic_robot2_x_arg = DeclareLaunchArgument(
        "traffic_robot2_x",
        default_value="10.0",
        description="Traffic robot 2 spawn x position.",
    )
    traffic_robot2_y_arg = DeclareLaunchArgument(
        "traffic_robot2_y",
        default_value="0.0",
        description="Traffic robot 2 spawn y position.",
    )

    # ---- Sub-launches ----
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "town.launch.py")
        ),
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "bridge.launch.py")
        ),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "navigation.launch.py")
        ),
    )

    executor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(executor_share, "launch", "mission_executor.launch.py")
        ),
    )

    gui_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gui_share, "launch", "gui.launch.py")
        ),
    )

    # ---- Traffic (delayed, conditional) ----
    # Override world_name to 'default' to match Suhaib's town.world.
    traffic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(traffic_share, "launch", "two_traffic_robots.launch.py")
        ),
        launch_arguments={
            "world_name": "default",
            "robot1_x": LaunchConfiguration("traffic_robot1_x"),
            "robot1_y": LaunchConfiguration("traffic_robot1_y"),
            "robot2_x": LaunchConfiguration("traffic_robot2_x"),
            "robot2_y": LaunchConfiguration("traffic_robot2_y"),
            "bounds_radius": "15.0",  # tighter wander radius → stays in busy zone
            "forward_speed": "1.0",
        }.items(),
    )
    delayed_traffic = TimerAction(
        period=15.0,
        actions=[traffic_launch],
        condition=IfCondition(LaunchConfiguration("use_traffic")),
    )

    # ---- Auto initial pose (delayed) ----
    initial_pose_yaml = (
        "{header: {frame_id: 'map'}, "
        "pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, "
        "covariance: ["
        "0.25, 0, 0, 0, 0, 0, "
        "0, 0.25, 0, 0, 0, 0, "
        "0, 0, 0, 0, 0, 0, "
        "0, 0, 0, 0, 0, 0, "
        "0, 0, 0, 0, 0, 0, "
        "0, 0, 0, 0, 0, 0.0685"
        "]}}"
    )
    init_pose_publish = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub",
            "--times", "3",
            "--rate", "1",
            "--qos-reliability", "best_effort",
            "/initialpose",
            "geometry_msgs/msg/PoseWithCovarianceStamped",
            initial_pose_yaml,
        ],
        output="screen",
    )
    delayed_init_pose = TimerAction(
        period=8.0,
        actions=[init_pose_publish],
        condition=IfCondition(LaunchConfiguration("auto_init_pose")),
    )

    # ---- Optional RViz ----
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        # args first
        use_rviz_arg,
        auto_init_pose_arg,
        use_traffic_arg,
        traffic_robot1_x_arg,
        traffic_robot1_y_arg,
        traffic_robot2_x_arg,
        traffic_robot2_y_arg,
        # core stack
        gazebo_launch,
        bridge_launch,
        navigation_launch,
        executor_launch,
        gui_launch,
        # delayed extras
        delayed_init_pose,
        delayed_traffic,
        # optional
        rviz_node,
    ])
