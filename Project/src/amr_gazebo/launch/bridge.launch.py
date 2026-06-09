from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            arguments=[
                '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                '/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
                '/world/default/model/vehicle_blue/link/chassis/sensor/front_cam/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/world/default/model/vehicle_blue/link/chassis/sensor/front_cam/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                '/world/default/model/vehicle_blue/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',
                '/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
                '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
            ],
            remappings=[
                ('/model/vehicle_blue/tf', '/tf'),
                ('/model/vehicle_blue/odometry', '/odom'),
            ]
        )
    ])
