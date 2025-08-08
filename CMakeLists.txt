cmake_minimum_required(VERSION 3.14.4)  # ROS 2 requires CMake 3.8+
project(rover)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ROS 2 Core ===========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # Replaces message_generation


set(msg_files
  "msg/ArmInputs.msg"
)

set(srv_files
  "srv/Corrections.srv"
#   "srv/GoToArmPos.srv"
#   "srv/SaveArmPos.srv"
)



# PCL Configuration ====================================================
find_package(PCL 1.12 REQUIRED)  # Use updated version for ROS 2
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Message/Service Generation ===========================================
# Declare message files
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/ArmInputs.msg"
#   # "msg/StateMsg.msg"
  
#   "srv/Corrections.srv"
#   "srv/GoToArmPos.srv"
#   "srv/SaveArmPos.srv"
#   # "srv/AddGoal.srv"
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#   geometry_msgs
#   visualization_msgs
# )

ament_export_dependencies(
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

# Package Configuration ================================================
# ament_package(
#   # INCLUDE_DIRS include
#   # LIBRARIES Rover
#   # CATKIN_DEPENDS removed in ROS 2
# )

add_executable(drive_sender_node rover/drive/src/drive_sender_node.cpp)
ament_target_dependencies(drive_sender_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_filters
)

# target_link_libraries(drive_sender_node PUBLIC
#   rclcpp::rclcpp
#   ${std_msgs_TARGETS}
#   ${sensor_msgs_TARGETS}
#   ${geometry_msgs_TARGETS}
#   ${message_filters_TARGETS}
# )

# DEPRECATED # Include Directories =================================================
# include_directories(
#   include
#   ${rosidl_include_directories}  # ROS 2 generated message headers
#   # $(OCTOMAP_INCLUDE_DIRS)
#   # rover/autonomy/scripts/obstacle_avoidance/include
# )

target_include_directories(drive_sender_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${rover}>"
)

# Executable Targets ==================================================
# Drive component
# add_executable(drive_sender_node rover/drive/src/drive_sender_node.cpp)
# target_link_libraries(drive_sender_node
#   ${rclcpp_LIBRARIES}
#   ${std_msgs_LIBRARIES}
# )


# Arm component
# add_executable(arm_point_cloud_assembler rover/arm/ros1/arm_point_cloud_assembler.cpp)
# target_link_libraries(arm_point_cloud_assembler
#   ${rclcpp_LIBRARIES}
#   ${sensor_msgs_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# Uncomment and update other targets as needed:
# add_executable(GNSSLocalization rover/autonomy/src/GNSSLocalization.cpp)
# target_link_libraries(GNSSLocalization ...)

# Install Targets =====================================================
install(TARGETS 
  drive_sender_node
#   arm_point_cloud_assembler
  # Add other executables here
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Python scripts installation (adjust for ROS 2)
# install(PROGRAMS
#   rover/autonomy/scripts/state_machine_node.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# Required ROS 2 macro
install(PROGRAMS
  scripts/manual_switch.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  rover/autonomy/scripts/imu_to_enu.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  rover/science/scripts/geniecamerapublisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_zed.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/manual_control
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  rover/science/scripts/microscopecam.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  rover/science/scripts/webcam.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  rover/science/scripts/webcam.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  rover/autonomy/scripts/science_arduino_transform.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/GUI/gui_pyqt.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/GUI/arm_task_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  rover/autonomy/scripts/led_light.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  rover/autonomy/scripts/pub_manual_indicator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY rover/autonomy/launch/lidar_slam
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)

install(
  DIRECTORY rover/autonomy/launch/mapping
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)

install(
  DIRECTORY rover/autonomy/launch/planner
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)

install(
  DIRECTORY rover/autonomy/launch/visual_odometry
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)


install(
  DIRECTORY rover/autonomy/launch
  DESTINATION share/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY rover/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml"
)

ament_package()