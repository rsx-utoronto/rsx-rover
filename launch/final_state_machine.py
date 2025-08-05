from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover',
            executable='final_state_machine.py',
            name='rover_state_machine',
            output='screen'
        )
    ])