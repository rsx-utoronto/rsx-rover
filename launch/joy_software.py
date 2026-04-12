from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='software',  # Equivalent to <group ns="software">
            parameters=[{'device_id': 1}],
            output='screen'
        )
    ])