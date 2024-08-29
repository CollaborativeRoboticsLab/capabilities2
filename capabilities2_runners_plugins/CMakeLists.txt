cmake_minimum_required(VERSION 3.10)
project(capabilities2_runner)

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

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/standard_runners.cpp
  # src/launch_runner.cpp
  # src/action_runner.cpp
  # src/nav_runner.cpp
  # src/moveit_runner.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  pluginlib
  capabilities2_msgs
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

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
