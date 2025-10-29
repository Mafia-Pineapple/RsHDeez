cmake_minimum_required(VERSION 3.8)
project(thermal_scan_trigger)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(thermal_scan_trigger_node src/thermal_scan_trigger.cpp)
ament_target_dependencies(thermal_scan_trigger_node
  rclcpp sensor_msgs std_msgs cv_bridge image_transport
)
target_link_libraries(thermal_scan_trigger_node ${OpenCV_LIBS})

target_compile_features(thermal_scan_trigger_node PRIVATE cxx_std_17)
target_link_libraries(thermal_scan_trigger_node stdc++fs)

install(TARGETS thermal_scan_trigger_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
