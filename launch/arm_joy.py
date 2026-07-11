from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='arm',
            parameters=[{'dev': '/dev/input/js0'}],
            output='screen'
        )
    ])