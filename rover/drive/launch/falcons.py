from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('rover'),
        # 'rover',
        'drive',
        'launch',
        'goose.yaml'
    ])

    return LaunchDescription([
        Node(
            package='ros_phoenix',
            executable='ros_phoenix_node',
            name='ros_phoenix',
            parameters=[config_file],
            output='screen',
            respawn=False
        ),
        Node(
            package='ros_phoenix',
            executable='falcon_motor_control_node',
            name='falcon_motor_control',
            parameters=[config_file],
            output='screen',
            respawn=False
        )
    ])