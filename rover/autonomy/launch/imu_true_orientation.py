from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # imu_to_enu node
        Node(
            package="rover",
            executable="imu_to_enu.py",
            name="imu_to_enu",
            output="screen"
        ),

        # imu_filter_madgwick node
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_node",
            name="imu_filter_madgwick",
            output="screen",
            parameters=[
                {"publish_tf": False},
                {"use_mag": True},
                {"world_frame": "enu"},
            ],
            remappings=[
                ("imu/mag", "/zed_node/imu/mag"),
                ("imu/data_raw", "/imu/enu"),
                ("imu/data", "/imu/orient"),
            ]
        ),
    ])