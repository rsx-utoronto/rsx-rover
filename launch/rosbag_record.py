from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', '/home/rsx/rover_ws/bags/bag',
                '/tf',
                '/tf_static',
                '/zed_node/depth/depth_registered',
                '/zed_node/depth/depth_registered/camera_info',
                '/zed_node/rgb/image_rect_color',
                '/zed_node/rgb/image_rect_color/compressed',
                '/zed_node/rgb/camera_info',
                '/zed_node/imu/data',
                '/zed_node/imu/mag',
                '/zed_node/point_cloud/cloud_registered',
                '/zed_node/odom',
                '/camera/depth/camera_info',
                '/camera/depth/image_rect_raw',
                '/camera/extrinsics/depth_to_color',
                '/camera/extrinsics/depth_to_infra1',
                '/camera/imu',
                '/camera/infra1/camera_info',
                '/camera/infra1/image_rect_raw',
                '/camera/color/camera_info',
                '/camera/color/image_raw',
                '/camera/color/image_raw/compressed',
                '/camera/depth/color/points',
                '/camera/gyro/imu_info',
                '/camera/pointcloud/parameter_updates',
                # Uncomment these if needed:
                # '/rtabmap/odom',
                # '/imu/orient',
                # '/calian_gnss/gps',
                # '/calian_gnss/gps_extended',
                # '/calian_gnss/rtcm_corrections',
                # '/calian_gnss/base_gps_extended'
            ],
            output='screen'
        )
    ])