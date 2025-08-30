from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch.actions import GroupAction
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        GroupAction([
            PushRosNamespace('zed2i/zed_node'),
            Node(
                package='stereo_image_proc',
                executable='stereo_image_proc',
                name='rectify_zed',
                remappings=[
                    ('left/image_raw', 'rgb/image_raw'),
                    ('left/camera_info', 'rgb/camera_info')
                ]
            )
        ])
    ])
