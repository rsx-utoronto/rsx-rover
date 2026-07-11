from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # Optional: Declare launch arguments (in case you want overrides later)
        DeclareLaunchArgument('rtabmap_args', default_value='--delete_db_on_start'),
        DeclareLaunchArgument('rgb_topic', default_value='/zed_node/rgb/image_rect_color'),
        DeclareLaunchArgument('depth_topic', default_value='/zed_node/depth/depth_registered'),
        DeclareLaunchArgument('camera_info_topic', default_value='/zed_node/rgb/camera_info'),
        DeclareLaunchArgument('imu_topic', default_value='/zed_node/imu/data'),
        DeclareLaunchArgument('wait_imu_to_init', default_value='true'),
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        DeclareLaunchArgument('approx_sync', default_value='false'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),

        # Include the main rtabmap launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            ])),
            launch_arguments={
                'rtabmap_args': LaunchConfiguration('rtabmap_args'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'wait_imu_to_init': LaunchConfiguration('wait_imu_to_init'),
                'frame_id': LaunchConfiguration('frame_id'),
                'approx_sync': LaunchConfiguration('approx_sync'),
                'rtabmap_viz': LaunchConfiguration('rtabmap_viz'),
                'rviz': LaunchConfiguration('rviz')
            }.items()
        )
    ])