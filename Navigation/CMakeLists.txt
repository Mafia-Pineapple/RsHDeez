cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Controller library
add_library(controller_lib src/controller.cpp)
ament_target_dependencies(controller_lib 
  rclcpp std_msgs geometry_msgs nav_msgs)

# Quadcopter executable
add_executable(quadcopter_node src/main.cpp src/quadcopter.cpp)
target_link_libraries(quadcopter_node controller_lib)
ament_target_dependencies(quadcopter_node 
  rclcpp std_msgs geometry_msgs nav_msgs std_srvs tf2 tf2_geometry_msgs)

# Goal publisher executable  
add_executable(goal_publisher_node src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher_node 
  rclcpp geometry_msgs)

# Fake world simulator (for testing without Gazebo)
add_executable(fake_world_node src/fake_world.cpp)
ament_target_dependencies(fake_world_node 
  rclcpp geometry_msgs nav_msgs)

# Install executables
install(TARGETS
  quadcopter_node
  goal_publisher_node
  fake_world_node
  DESTINATION lib/${PROJECT_NAME})

# Install directories
install(DIRECTORY
  launch
  worlds
  include
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()