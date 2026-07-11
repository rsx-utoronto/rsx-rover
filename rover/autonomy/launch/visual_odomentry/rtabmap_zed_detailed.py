from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindPackageShare
import os

def generate_launch_description():
    # Declare arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Enable RVIZ')
    rtabmap_viz_arg = DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Enable RTAB-Map visualization')

    use_sim_time = {'use_sim_time': True}

    return LaunchDescription([
        rviz_arg,
        rtabmap_viz_arg,

        # Stereo Odometry Node
        Node(
            package='rtabmap_odom',
            executable='stereo_odometry',
            name='stereo_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'imu_topic': '/zed2i/zed_node/imu/data',
                'wait_imu_to_init': False,
                'Odom/Strategy': '0',
                'Odom/EstimationType': '1',
                'Odom/MinInliers': '10',
                'Odom/RoiRatios': '0.03 0.03 0.04 0.04',
                'Odom/MaxDepth': '10',
                'OdomBow/NNDR': '0.8',
                'Odom/MaxFeatures': '1000',
                'Odom/FillInfoData': LaunchConfiguration('rtabmap_viz'),
                'GFTT/MinDistance': '10',
                'GFTT/QualityLevel': '0.00001',
                **use_sim_time
            }],
            remappings=[
                ('left/image_rect', '/zed2i/zed_node/rgb/image_rect_color'),
                ('right/image_rect', '/zed2i/zed_node/right/image_rect_color'),
                ('left/camera_info', '/zed2i/zed_node/rgb/camera_info'),
                ('right/camera_info', '/zed2i/zed_node/right/camera_info'),
                ('odom', '/stereo_odometry'),
            ]
        ),

        # RTAB-Map Core
        GroupAction([
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                output='screen',
                arguments=['--delete_db_on_start'],
                namespace='rtabmap',
                parameters=[{
                    'frame_id': 'base_link',
                    'subscribe_stereo': True,
                    'subscribe_depth': False,
                    'subscribe_scan': False,
                    'subscribe_scan_cloud': False,
                    'queue_size': 30,
                    'Rtabmap/TimeThr': '700',
                    'Rtabmap/DetectionRate': '1',
                    'Kp/WordsPerImage': '200',
                    'Kp/RoiRatios': '0.03 0.03 0.04 0.04',
                    'Kp/DetectorStrategy': '0',
                    'Kp/NNStrategy': '1',
                    'SURF/HessianThreshold': '1000',
                    'LccBow/MinInliers': '10',
                    'LccBow/EstimationType': '1',
                    'LccReextract/Activated': 'true',
                    'LccReextract/MaxWords': '500',
                    'LccReextract/MaxDepth': '10',
                    **use_sim_time
                }],
                remappings=[
                    ('left/image_rect', '/zed2i/zed_node/rgb/image_rect_color'),
                    ('right/image_rect', '/zed2i/zed_node/right/image_rect_color'),
                    ('left/camera_info', '/zed2i/zed_node/rgb/camera_info'),
                    ('right/camera_info', '/zed2i/zed_node/right/camera_info'),
                    ('depth/image', '/zed2i/zed_node/depth/depth_registered'),
                    ('scan', '/scan'),
                    ('scan_cloud', '/velodyne_points'),
                    ('odom', '/stereo_odometry'),
                ]
            ),

            # RTAB-Map Visualization
            Node(
                condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
                package='rtabmap_viz',
                executable='rtabmap_viz',
                name='rtabmap_viz',
                namespace='rtabmap',
                output='screen',
                arguments=['-d', PathJoinSubstitution([
                    FindPackageShare('rtabmap_ros'),
                    'launch', 'config', 'rgbd_gui.ini'
                ])],
                parameters=[{
                    'subscribe_stereo': True,
                    'subscribe_odom_info': True,
                    'queue_size': 10,
                    'frame_id': 'base_link',
                    **use_sim_time
                }],
                remappings=[
                    ('left/image_rect', '/zed2i/zed_node/rgb/image_rect_color'),
                    ('right/image_rect', '/zed2i/zed_node/right/image_rect_color'),
                    ('left/camera_info', '/zed2i/zed_node/rgb/camera_info'),
                    ('right/camera_info', '/zed2i/zed_node/right/camera_info'),
                    ('odom_info', '/odom_info'),
                    ('odom', '/stereo_odometry'),
                    ('mapData', 'mapData'),
                ]
            )
        ]),

        # RViz (optional)
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz',
            executable='rviz',
            name='rviz',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('rtabmap_demos'),
                'launch', 'config', 'demo_stereo_outdoor.rviz'
            ])]
        )
    ])