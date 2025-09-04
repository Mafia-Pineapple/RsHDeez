cmake_minimum_required(VERSION 3.8)
project(navigation)

# ---------- Build options ----------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

# ---------- Dependencies ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# ---------- Include dirs ----------
# Public headers live under include/navigation/*.h
include_directories(include)

# ---------- Library (core logic) ----------
add_library(${PROJECT_NAME}_lib
  src/controller.cpp
  src/quadcopter.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs tf2 tf2_geometry_msgs
)

# ---------- Executable (node entrypoint) ----------
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp geometry_msgs nav_msgs std_msgs std_srvs tf2 tf2_geometry_msgs
)
add_executable(fake_world src/fake_world.cpp)
ament_target_dependencies(fake_world rclcpp geometry_msgs nav_msgs)
install(TARGETS fake_world RUNTIME DESTINATION lib/${PROJECT_NAME})

add_executable(gz_adapter src/gz_adapter.cpp)
ament_target_dependencies(gz_adapter rclcpp geometry_msgs nav_msgs ros_gz_interfaces)
install(TARGETS gz_adapter RUNTIME DESTINATION lib/${PROJECT_NAME})

# ---------- Installation ----------
# Binaries / libraries
install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Public headers
install(DIRECTORY include/
  DESTINATION include
)

# Optional resources (only install if present to avoid CMake errors)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/urdf)
  install(DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
  )
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/worlds)
  install(DIRECTORY worlds/
    DESTINATION share/${PROJECT_NAME}/worlds
  )
endif()

# ---------- Export ----------
ament_package()
