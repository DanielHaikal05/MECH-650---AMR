# AMR Project

ROS 2 simulation workspace for an autonomous mobile robot in a Gazebo town environment. The project supports two main workflows: mapping the town and running the full navigation system.

## Setup

Install the basic ROS 2 build tools:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
```

Clone, install dependencies, and build:

```bash
git clone https://github.com/Suhaib-AbuRaidah/AMR_Project.git
cd AMR_Project
source /opt/ros/<ros-distro>/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Replace `<ros-distro>` with your ROS 2 distribution, for example `humble` or `jazzy`.

## Mapping

Launch Gazebo, the bridge, SLAM, exploration, and RViz:

```bash
ros2 launch amr_mapping mapping_full.launch.py
```

Use this when you want to generate or update the map used by navigation.

## Navigation

Launch the full system with Gazebo, bridge, Nav2, GUI, RViz, and traffic robots:

```bash
ros2 launch amr_navigation full_system.launch.py use_rviz:=true use_traffic:=true
```

Use this when you want to run missions and navigate through the town.

## Notes

Source the workspace in every new terminal:

```bash
source /opt/ros/<ros-distro>/setup.bash
source install/setup.bash
```

Main world file:

```text
src/amr_gazebo/worlds/town.world
```
