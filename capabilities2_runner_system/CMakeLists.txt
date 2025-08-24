cmake_minimum_required(VERSION 3.8)
project(capabilities2_runner_system)

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
find_package(capabilities2_msgs REQUIRED)
find_package(capabilities2_runner REQUIRED)
find_package(capabilities2_utils REQUIRED)
find_package(fabric_msgs REQUIRED)
find_package(event_logger REQUIRED)
find_package(event_logger_msgs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)  # provided by tinyxml2 upstream, or tinyxml2_vendor

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/capabilities2_runner.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  pluginlib
  capabilities2_msgs
  fabric_msgs
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

install(DIRECTORY interfaces
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY providers
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
