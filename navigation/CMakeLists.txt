cmake_minimum_required(VERSION 3.8)
project(navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# Main quadcopter node with integrated decision making
add_executable(quadcopter_node
  src/main.cpp
  src/quadcopter.cpp
  src/controller_base.cpp
  src/decision_making.cpp
)



ament_target_dependencies(quadcopter_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

# Other utility nodes
add_executable(agl_parser src/agl_parser.cpp)
ament_target_dependencies(agl_parser
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
)

add_executable(pose_to_odom src/pose_to_odom.cpp)
ament_target_dependencies(pose_to_odom
  rclcpp
  geometry_msgs
  nav_msgs
)

add_executable(goal_publisher_node src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher_node
  rclcpp
  geometry_msgs
)

add_executable(fake_world_node src/fake_world.cpp)
ament_target_dependencies(fake_world_node
  rclcpp
  geometry_msgs
  nav_msgs
)

add_executable(odometry_offset_node src/odometry_offset.cpp)
ament_target_dependencies(odometry_offset_node 
  rclcpp nav_msgs geometry_msgs tf2_ros)


install(TARGETS
  quadcopter_node
  agl_parser
  pose_to_odom
  goal_publisher_node
  fake_world_node
  odometry_offset_node  # ADD THIS
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  include
  models
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()