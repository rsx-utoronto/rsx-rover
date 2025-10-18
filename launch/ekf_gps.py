from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([

        SetLaunchConfiguration('use_sim_time', 'false'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_broadcaster_base_to_gps',
            arguments=['-0.2', '-0.355', '0.18', '0', '0', '0', '1', 'base_link', 'gps'],
        ),

        Node(
            package='rover',
            executable='imu_to_enu.py',
            name='imu_to_enu',
            output='screen'
        ),

        Node(
            package='rover',
            executable='gps_to_pose.py',
            name='gps_to_pose',
            output='screen'
        ),

        Node(
            package='rover',
            executable='ekf_odom_to_pose.py',
            name='ekf_odom_to_pose',
            output='screen'
        ),

        Node(
            package='rover',
            executable='override_covariance.py',
            name='override_covariance',
            output='screen',
            parameters=[{
                'imu_orien_cov_multiplier': 10.0,
                'imu_ang_vel_cov_multiplier': 10.0,
                'imu_lin_acc_cov_multiplier': 10.0,
                'gps_cov_multiplier': 1.0,
                'odom_pose_cov_multiplier': 100000.0,
                'odom_twist_cov_multiplier': 10000.0
            }]
        ),

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'publish_tf': False,
                'use_mag': True,
                'world_frame': 'enu',
                'orientation_stddev': 0.2
            }],
            remappings=[
                ('imu/mag', '/zed_node/imu/mag'),
                ('imu/data_raw', '/imu/enu'),
                ('imu/data', '/imu/orient')
            ]
        ),

        Node(
            package='rover',
            executable='heading_filter.py',
            name='heading_filter',
            output='screen',
            parameters=[{
                'magnetic_declination_radians': -0.177764053,
                'armlength': 1.65
            }]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[{
                'publish_tf': True,
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'map_frame': 'map',
                'world_frame': 'odom',
                'frequency': 15.0,
                'sensor_timeout': 0.2,
                'two_d_mode': False,
                'print_diagnostics': True,
                'imu0': '/imu/orient/override',
                'imu0_differential': False,
                'imu0_relative': False,
                'imu0_queue_size': 10,
                'imu0_remove_gravitational_acceleration': True,
                'imu0_config': [False, False, False,
                                True, True, True,
                                False, False, False,
                                False, False, False,
                                False, False, False],
                "odom0": "/rtabmap/odom/override",
                "odom0_differential": False,
                "odom0_relative": False,
                "odom0_queue_size": 10,
                "odom0_config": [True, True, True,
                                 False, False, False, 
                                 False, False, False,
                                 False, False, False, 
                                 False, False, False],
            }],
            remappings=[('odometry/filtered', '/odom/ekf')]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            parameters=[{
                'publish_tf': True,
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'map_frame': 'map',
                'world_frame': 'map',
                'frequency': 5.0,
                'sensor_timeout': 2.0,
                'two_d_mode': False,
                'print_diagnostics': True,
                'imu0': '/imu/orient/override',
                'imu0_differential': False,
                'imu0_relative': False,
                'imu0_queue_size': 10,
                'imu0_remove_gravitational_acceleration': True,
                'imu0_config': [False, False, False,
                                True, True, True,
                                False, False, False,
                                False, False, False,
                                False, False, False],
                "odom0": "/rtabmap/odom/override", # added odom0 input
                "odom0_differential": False,
                "odom0_relative": False,
                "odom0_queue_size": 10,
                "odom0_config": [True, True, True, 
                                 False, False, False,
                                 False, False, False,
                                 False, False, False, 
                                 False, False, False],
                'odom1': '/navsat/gps',
                'odom1_differential': False,
                'odom1_relative': False,
                'odom1_queue_size': 10,
                'odom1_config': [True, True, True,
                                 True, True, True,
                                 True, True, True,
                                 True, True, True,
                                 False, False, False]
            }],
            remappings=[
                ('odometry/filtered', '/map/ekf')
                ]
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[{
                'frequency': 5.0,
                'magnetic_declination_radians': -0.177764053,
                'use_odometry_yaw': False,
                'yaw_offset': 0.0,
                # had to comment out these lines to make the node sub to imu
                # 'wait_for_datum': True,
                # 'datum': [43.660517, -79.396553, 0.0] # deleted frame transformation paras for compatibility
            }],
            remappings=[
                ('imu', '/fused_heading'),
                ('gps/fix', '/calian_gnss/gps/override'),
                ('odometry/filtered', '/map/ekf'),

                ('odometry/gps', '/navsat/gps') # feedback to ekf
            ]
        ),
    ])