from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('rover'),
        'rover',
        'drive',
        'launch',
        'goose.yaml'
    ])

    config = os.path.join(
        get_package_share_directory('rover'),
        'config',
        'goose.yaml'
    )

    return LaunchDescription([
        Node(
            package='ros_phoenix',
            executable='ros_phoenix_node',
            name='ros_phoenix',
            parameters=[config],
            output='screen',
            respawn=False
        ),
        Node(
            package='ros_phoenix',
            executable='falcon_motor_control_node',
            name='falcon_motor_control',
            parameters=[config],
            output='screen',
            respawn=False
        )
    ])