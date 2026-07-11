#this package does not work in ros1
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os



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
    

    move_base_node = Node(
        package='move_base',
        executable='move_base',
        name='move_base',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, 'costmap_common.yaml']),
            PathJoinSubstitution([config_path, 'costmap_common.yaml']),
            PathJoinSubstitution([config_path, 'local_costmap.yaml']),
            # Conditionally load global_costmap.yaml if no_static_map is false
            # ROS 2 launch doesn't support 'unless', so this must be done via Python condition or separate launch files
            # Here we load unconditionally; for conditional loading, a Python if statement is required.
            PathJoinSubstitution([config_path, 'global_costmap.yaml']),
            PathJoinSubstitution([config_path, 'move_base.yaml']),
            {'base_global_planner': 'navfn/NavfnROS'},
            {'base_local_planner': 'dwa_local_planner/DWAPlannerROS'},
            {'controller_frequency': 5.0},
            {'controller_patience': 15.0},
        ],
        remappings=[
            ('cmd_vel', command_velocity_topic),
            ('odom', odom_topic),
            ('amcl_pose', pose_topic),
            ('map', map_topic),
        ],
    )

    map_server_node = Node(
        package='map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        arguments=[str(PathJoinSubstitution([maps_dir, 'white.yaml']))],
        parameters=[{'frame_id': 'map'}],
        remappings=[('map', map_topic)],
    )

    rviz_node = Node(
        package='rviz2',  # In ROS2 it's rviz2, not rviz
        executable='rviz2',
        name='rviz_move_base',
        arguments=['-d', str(PathJoinSubstitution([config_dir, 'move_base.rviz']))],
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
        move_base_node,
        map_server_node,
        rviz_node,
    ])