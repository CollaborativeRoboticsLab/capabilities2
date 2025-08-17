cmake_minimum_required(VERSION 3.10)
project(capabilities2_runner_nav2)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(capabilities2_msgs REQUIRED)
find_package(capabilities2_runner REQUIRED)
find_package(capabilities2_utils REQUIRED)
find_package(event_logger REQUIRED)
find_package(event_logger_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)  # provided by tinyxml2 upstream, or tinyxml2_vendor

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/nav2_runners.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  pluginlib
  nav2_msgs
  tf2
  tf2_ros
  geometry_msgs
  capabilities2_msgs
  capabilities2_runner
  capabilities2_utils
  event_logger
  event_logger_msgs
  TinyXML2
)

pluginlib_export_plugin_description_file(capabilities2_runner plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
