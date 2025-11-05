cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================
# DEPENDENCIES
# ============================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

# ============================================
# CORE NODES - Low-level drone control
# ============================================
# quadcopter_node - Main flight controller
add_executable(quadcopter_node
  src/main.cpp
  src/core/quadcopter.cpp
  src/core/controller_base.cpp
  src/core/decision_making.cpp
  # REMOVE simpleController.cpp - has errors
)
ament_target_dependencies(quadcopter_node
  rclcpp std_msgs geometry_msgs nav_msgs std_srvs sensor_msgs
  tf2 tf2_geometry_msgs
)

# ============================================
# EXPLORATION NODES - Autonomous mapping
# ============================================
# smooth_explorer_node - Collision-avoiding continuous explorer
add_executable(smooth_explorer_node src/exploration/smooth_explorer.cpp)
ament_target_dependencies(smooth_explorer_node 
  rclcpp geometry_msgs nav_msgs sensor_msgs)

# autonomous_explorer_node - Goal-based explorer
add_executable(autonomous_explorer_node src/exploration/autonomous_explorer.cpp)
ament_target_dependencies(autonomous_explorer_node 
  rclcpp geometry_msgs nav_msgs sensor_msgs std_srvs)

# ============================================
# DETECTION NODES - Animal/object detection
# ============================================
# bear_detector_node - Detects white bear using camera + depth
add_executable(bear_detector_node src/detection/bear_detector.cpp)
ament_target_dependencies(bear_detector_node 
  rclcpp geometry_msgs sensor_msgs std_msgs visualization_msgs 
  tf2_ros tf2_geometry_msgs cv_bridge)
target_link_libraries(bear_detector_node ${OpenCV_LIBS})

# ============================================
# NAVIGATION NODES - 3D path planning
# ============================================
# COMMENTED OUT - needs octomap_msgs dependency
# add_executable(simple_3d_navigator_node src/navigation/simple_3d_navigator.cpp)
# ament_target_dependencies(simple_3d_navigator_node 
#   rclcpp geometry_msgs nav_msgs sensor_msgs tf2_ros tf2_geometry_msgs)

# ============================================
# UTILITY NODES - Tools and converters
# ============================================
# agl_parser - Processes altitude above ground sensor
add_executable(agl_parser src/utils/agl_parser.cpp)
ament_target_dependencies(agl_parser
  rclcpp sensor_msgs std_msgs nav_msgs)

# goal_publisher_node - Manual goal setting tool
add_executable(goal_publisher_node src/utils/goal_publisher.cpp)
ament_target_dependencies(goal_publisher_node
  rclcpp geometry_msgs)

# fake_world_node - Simple dynamics simulator (testing)
add_executable(fake_world_node src/utils/fake_world.cpp)
ament_target_dependencies(fake_world_node
  rclcpp geometry_msgs nav_msgs)

# odometry_offset_node - Corrects odometry drift
add_executable(odometry_offset_node src/utils/odometry_offset.cpp)
ament_target_dependencies(odometry_offset_node 
  rclcpp nav_msgs geometry_msgs tf2_ros)

# laserscan_to_pointcloud_node - Converts LaserScan to PointCloud2
add_executable(laserscan_to_pointcloud_node src/utils/laserscan_to_pointcloud.cpp)
ament_target_dependencies(laserscan_to_pointcloud_node 
  rclcpp sensor_msgs laser_geometry)

# COMMENTED OUT - needs rclcpp_action dependency
# add_executable(local_goal_publisher_node src/utils/local_goal_publisher.cpp)
# ament_target_dependencies(local_goal_publisher_node
#   rclcpp geometry_msgs)

# ============================================
# INSTALL
# ============================================
install(TARGETS
  # Core
  quadcopter_node
  
  # Exploration
  smooth_explorer_node
  autonomous_explorer_node
  
  # Detection
  bear_detector_node
  
  # Utils
  agl_parser
  goal_publisher_node
  fake_world_node
  odometry_offset_node
  laserscan_to_pointcloud_node
  
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  include
  models
  DESTINATION share/${PROJECT_NAME}
)

# ============================================
# TESTING
# ============================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()