
from ros2_launch_file_migrator import LaunchFileMigrator


zed_odom = ("<launch>\n"
                             '<node pkg="tf" type="static_transform_publisher" name="zed_broadcaster" args="0.33 0.0 0.0 0 0 0 1 base_link zed2_base_link 100" />\n'
                             '<include file="$(find zed_wrapper)/launch/zed_no_tf.launch"/>\n'
                             '<node pkg="pointcloud_to_laserscan" type="pointcloud_to_laserscan_node" name="collapse_pointcloud" output="screen" >\n'
                             '<param name="max_height" value="1" />\n'
                             '<param name="min_height" value="-0.5" />\n'
                             '<param name="range_min" value="0.75" />\n'
                             '<param name="range_max" value="5.0" />\n'
                             '<remap from="cloud_in" to="/zed_node/point_cloud/cloud_registered"/>\n'
                             '</node>\n'
                             '</launch>')



amd_sci = ("<launch>\n"
           '<node pkg="rover" type="manual_control" name="rover_manual_control" output="screen"/>\n'
           '<include file="$(find rover)/launch/rs_multiple_devices.launch"/>\n'
           '<node pkg="rover" type="imu_to_enu.py" name="imu_to_enu" output="screen" />\n'
           '<node pkg="imu_filter_madgwick" type="imu_filter_node" name="imu_filter_madgwick" output="screen" >\n'
           '<param name="publish_tf" value="false" />\n'
           '<param name="use_mag" value="true" />\n'
           '<param name="world_frame" value="enu" />\n'
           '<param name="orientation_stddev" value="0.2" />\n'
           '<remap from="imu/mag" to="/zed_node/imu/mag" />\n'
           '<remap from="imu/data_raw" to="/imu/enu" />\n'
           '<remap from="imu/data" to="/imu/orient" />\n'
           '</node>\n'
           "<include file='$(find calian_gnss_ros2)/launch/moving_baseline_rtk.launch'/>\n"
           '<node pkg="rover" type="check_zed.py" name="check_zed" output="screen" />\n'
           '<node pkg="rover" type="geniecamerapublisher.py" name="geniecam_publisher" output="screen" />\n'
           '<node pkg="rover" type="microscopecam.py" name="microscopecam" output="screen" />\n'
           '<node pkg="rover" type="webcam.py" name="webcam_node" output="screen" />\n'
           '<node pkg="rover" type="science_arduino_transform.py" name="science_arduino_transform" output="screen" />\n'
           '</launch>')

arm_joy = ("<launch>\n"
           '<group ns="arm">\n'
           '<node pkg="joy" type="joy_node" name="joy_node">\n'
           '<param name="dev" value="/dev/input/js0"/>\n'
           '</node>\n'
           '</group>\n'
           '</launch>\n')

base_arm_tasks = ('<launch>\n'
                  '<node pkg="rover" type="arm_task_gui.py" name="rover_gui" output="screen"/>\n'
                  "<include file='$(find rover)/launch/joy.launch'/>\n"
                  '</launch>')

base_auto = ("<launch>\n"
             '<node pkg="rover" type="gui_pyqt.py" name="rover_gui" output="screen"/>\n'
             "<include file='$(find rover)/launch/joy.launch'/>\n"
             '</launch>')


base_sci = ('<launch>\n'
            '<node pkg="rover" type="science_gui.py" name="rover_gui" output="screen"/>\n'
            "<include file='$(find rover)/launch/joy.launch'/>\n"
            '</launch>')

state_mach = ("<launch>\n"
              '<node pkg="rover" type="final_state_machine.py" name="rover_state_machine" output="screen" />\n'
              '</launch>\n')

joy_software = ('<launch>\n'
                '<group ns="software">\n'
                '<node pkg="joy" type="joy_node" name="joy_node">\n'
                '<param name="dev" value="/dev/input/js1"/>\n'
                '</node>\n'
                '</group>\n'
                '</launch>\n')

joy = ('<launch>\n'
       '<group ns="arm">\n'
       '<node pkg="joy" type="joy_node" name="joy_node">\n'
       '<param name="dev" value="/dev/input/js1"/>\n'
       '</node>\n'
       '</group>\n'
       '<group ns="software">\n'
       '<node pkg="joy" type="joy_node" name="joy_node">\n'
       '<param name="dev" value="/dev/input/js0"/>\n'
       '</node>\n'
       '</group>\n'
       '</launch>\n')



real_sense = ('<launch>\n' 
              '<include file="$(find realsense2_camera)/launch/rs_camera.launch" >\n'
              '<arg name="enable_infra"  value="true"/>\n'
              '<arg name="enable_infra1"  value="true"/>\n'
              '<arg name="enable_infra2"  value="true"/>\n'
              '<arg name="depth_width"   value="640"/>\n'
              '<arg name="depth_height"  value="480"/>\n' 
              '<arg name="depth_fps"    value="15"/>\n' 
              '<arg name="color_width"   value="640"/>\n'
              '<arg name="color_height"  value="480"/>\n'
              '<arg name="color_fps"    value="15"/>\n'
              '<arg name="infra_width"   value="640"/>\n'
              '<arg name="infra_height"  value="480"/>\n'
              '<arg name="infra_fps"    value="15"/>\n'
              '<arg name="infra_rgb"   value="false"/>\n'
              '<arg name="enable_sync" value="true"/>\n'
              '<arg name="align_depth" value="true"/>\n'
              '<arg name="enable_accel" value="true"/>\n'
              '<arg name="enable_gyro"  value="true"/>\n'
              '<arg name="enable_pointcloud" value="true"/>\n'
              '<arg name="unite_imu_method" value="linear_interpolation"/>\n'
              '</include>\n'
              '</launch>\n')


rosbag_record = ('<launch>\n'
                 '<node pkg="rosbag" type="record" name="rosbag_record" cwd="node"\n'
                 'args="record -o /home/rsx/rover_ws/bags/bag\n'
                 '/tf\n'
                 '/tf_static\n'
                 '/zed_node/depth/depth_registered\n'
                 '/zed_node/depth/depth_registered/camera_info\n'
                 '/zed_node/rgb/image_rect_color\n'
                 '/zed_node/rgb/image_rect_color/compressed\n'
                 '/zed_node/rgb/camera_info\n'
                 '/zed_node/imu/data\n'
                 '/zed_node/imu/mag\n'
                 '/zed_node/point_cloud/cloud_registered\n'
                 '/zed_node/odom\n'
                 '/camera/depth/camera_info\n'
                 '/camera/depth/image_rect_raw\n'
                 '/camera/extrinsics/depth_to_color\n'
                 '/camera/extrinsics/depth_to_infra1\n'
                 '/camera/imu\n'
                 '/camera/infra1/camera_info\n'
                 '/camera/infra1/image_rect_raw\n'
                 '/camera/color/camera_info\n'
                 '/camera/color/image_raw\n'
                 '/camera/color/image_raw/compressed\n'
                 '/camera/depth/color/points\n'
                 '/camera/gyro/imu_info\n'
                 '/camera/pointcloud/parameter_updates\n'
                 '-b 3072"/>\n'
                 '</launch>')








converted_code =  LaunchFileMigrator.migrate(rosbag_record)

print(converted_code)