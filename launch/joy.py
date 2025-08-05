from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([

        # Group for arm namespace with /dev/input/js1
        GroupAction([
            PushRosNamespace('arm'),
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{'dev': '/dev/input/js1'}]
            )
        ]),

        # Group for software namespace with /dev/input/js0
        GroupAction([
            PushRosNamespace('software'),
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{'dev': '/dev/input/js0'}]
            )
        ])
    ])