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
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(custom_msgs REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})

# ============================================
# CORE NODES - Low-level drone control
# ============================================
# quadcopter_node - Main flight controller
add_executable(quadcopter_node
  src/main.cpp
  src/core/quadcopter.cpp
  src/core/controller_base.cpp
  src/core/decision_making.cpp
)
ament_target_dependencies(quadcopter_node
  rclcpp std_msgs geometry_msgs nav_msgs std_srvs sensor_msgs
  tf2 tf2_geometry_msgs custom_msgs
)

# ============================================
# EXPLORATION NODES - Autonomous mapping
# ============================================
# smooth_explorer_node - Collision-avoiding continuous explorer

  # grid_explorer_node - Systematic grid-based explorer with AGL
add_executable(grid_explorer_node src/exploration/grid_explorer.cpp)
ament_target_dependencies(grid_explorer_node 
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs custom_msgs)


# ============================================
# DETECTION NODES - Animal/object detection
# ============================================
# bear_detector_node - Detects white bear using thermal camera + depth


# ============================================
# UTILITY NODES - Tools and converters
# ============================================
# agl_parser - Processes altitude above ground sensor
add_executable(agl_parser src/utils/agl_parser.cpp)
ament_target_dependencies(agl_parser
  rclcpp sensor_msgs std_msgs nav_msgs custom_msgs)





# odometry_offset_node - Corrects odometry drift
add_executable(odometry_offset_node src/utils/odometry_offset.cpp)
ament_target_dependencies(odometry_offset_node 
  rclcpp nav_msgs geometry_msgs tf2_ros custom_msgs)


# ============================================
# INSTALL
# ============================================
install(TARGETS
  # Core
  quadcopter_node
  


  


  grid_explorer_node
  # Utils
  agl_parser
  odometry_offset_node
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
