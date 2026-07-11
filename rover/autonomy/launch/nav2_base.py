from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')


def generate_launch_description():
    # Declare launch arguments with defaults
    no_static_map_arg = DeclareLaunchArgument('no_static_map', default_value='false')
    command_velocity_topic_arg = DeclareLaunchArgument('command_velocity_topic', default_value='/drive')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='rtabmap/odom')
    pose_topic_arg = DeclareLaunchArgument('pose_topic', default_value='rtabmap/odom')
    use_map_topic_arg = DeclareLaunchArgument('use_map_topic', default_value='true')
    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value='/scan_collapsed')
    map_topic_arg = DeclareLaunchArgument('map_topic', default_value='/projected_map')
    global_costmap_topic_arg = DeclareLaunchArgument('global_costmap_topic', default_value='/move_base/global_costmap/costmap')

    no_static_map = LaunchConfiguration('no_static_map')
    command_velocity_topic = LaunchConfiguration('command_velocity_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    pose_topic = LaunchConfiguration('pose_topic')
    use_map_topic = LaunchConfiguration('use_map_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    map_topic = LaunchConfiguration('map_topic')
    global_costmap_topic = LaunchConfiguration('global_costmap_topic')

    rover_share_dir = FindPackageShare('rover')
    print(f"Rover share directory: {rover_share_dir}")
    config_dir = PathJoinSubstitution([rover_share_dir,  'autonomy', 'config'])
    maps_dir = PathJoinSubstitution([rover_share_dir, 'autonomy', 'maps'])
    
    rover_share_dir = get_package_share_directory('rover')
    config_path = os.path.join(rover_share_dir, 'autonomy', 'config')
    
    print(f"Config path: {config_path}")
    
    maps_dir_str = os.path.join(rover_share_dir, 'autonomy', 'maps')
    config_path_str = os.path.join(rover_share_dir, 'autonomy', 'config')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'map': os.path.join(maps_dir_str, 'white.yaml'),
            'use_sim_time': 'false',
            'params_file': os.path.join(config_path_str, 'nav2_params.yaml'),
        }.items()
    )

 
    map_server_node = Node(
        package='map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        arguments=[PathJoinSubstitution([maps_dir, 'white.yaml'])],
        parameters=[{'frame_id': 'map'}],
        remappings=[('map', map_topic)],
    )

    rviz_node = Node(
        package='rviz2',  # In ROS2 it's rviz2, not rviz
        executable='rviz2',
        name='rviz_move_base',
        arguments=['-d', PathJoinSubstitution([config_dir, 'move_base.rviz'])],
    )

    return LaunchDescription([
    no_static_map_arg,
    command_velocity_topic_arg,
    odom_topic_arg,
    pose_topic_arg,
    use_map_topic_arg,
    scan_topic_arg,
    map_topic_arg,
    global_costmap_topic_arg,
    nav2_launch,      # <-- use nav2 launch here
    map_server_node,  # optional, might be included by nav2 already
    rviz_node,        # optional, depending on your setup
    ])
