from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        # Joy node in 'auto' namespace
        GroupAction([
            PushRosNamespace('auto'),
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{'dev': '/dev/input/js0'}]
            )
        ]),

        # Manual switch node (assuming it's a Python script with executable permission)
        Node(
            package='rover',
            executable='manual_switch.py',
            name='auto_switch',
            output='screen'
        )
    ])
