cmake_minimum_required(VERSION 3.8)
project(capabilities2_test_suite)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(bondcpp REQUIRED)
find_package(capabilities2_runner REQUIRED)
find_package(capabilities2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(
  include
  ${UUID_INCLUDE_DIRS}
)

############################################################################
# Test plugins
############################################################################

add_library(${PROJECT_NAME} SHARED
  src/test_runners.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  pluginlib
  capabilities2_msgs
  capabilities2_runner
  capabilities2_events
)

pluginlib_export_plugin_description_file(capabilities2_runner plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY interfaces
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY providers
  DESTINATION share/${PROJECT_NAME}
)


############################################################################
# Test Node Executables
############################################################################

add_executable(test_action_runner_node
  src/test_action_runner_node.cpp
)

ament_target_dependencies(test_action_runner_node
  rclcpp
  rclcpp_action
  bondcpp
  pluginlib
  capabilities2_runner
  capabilities2_msgs
)

install(TARGETS test_action_runner_node
  DESTINATION lib/${PROJECT_NAME}
)

############################################################################
# Test Interfaces
############################################################################

set(msg_files
)

set(srv_files
)

set(action_files
  "action/Fibonacci.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

############################################################################
# Miscellaneous installations
############################################################################

install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
