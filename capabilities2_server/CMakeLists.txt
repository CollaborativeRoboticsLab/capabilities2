cmake_minimum_required(VERSION 3.10)
project(capabilities2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(capabilities2_msgs REQUIRED)

include_directories(include)

add_executable(capabilities_server src/capabilities2_server_node.cpp)
ament_target_dependencies(capabilities_server rclcpp, capabilities2_msgs)

install(TARGETS
  capabilities_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
