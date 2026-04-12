from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction, SetEnvironmentVariable
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Resolve path to the Fast DDS LAN-only profile and set env vars
    pkg_share = get_package_share_directory('rover')
    fastdds_profile = os.path.join(pkg_share, 'config', 'fastdds_lan_only.xml')

    return LaunchDescription([
        # Force Fast DDS and restrict to LAN interface via profiles file
        # SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_fastrtps_cpp'),
        # SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='0'),
        # Support both env names used by different Fast DDS versions
        # SetEnvironmentVariable(name='FASTDDS_DEFAULT_PROFILES_FILE', value=fastdds_profile),
        # SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=fastdds_profile),

        # Group for arm namespace with /dev/input/js1
        GroupAction([
            PushRosNamespace('arm'),
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{'device_id': 1}]
            )
        ]),

        # Group for software namespace with /dev/input/js0
        GroupAction([
            PushRosNamespace('software'),
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{'device_id': 0}]
            )
        ])
    ])